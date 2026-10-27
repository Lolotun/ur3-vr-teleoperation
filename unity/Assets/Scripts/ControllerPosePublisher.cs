using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.XR;

// Pose input only: robot anchors and target calculations live in ROS.
public class ControllerPosePublisher : MonoBehaviour
{
    public Transform rightController;
    public Transform robotBase;
    public ControllerTwistPublisher legacyPublisher;
    [Tooltip("Enable for RTDE pose control. Disable for legacy speedl / Servo simulation.")]
    public bool poseMode = false;
    public float publishHz = 30f;
    public string frameId = "vr_robot_base";
    private ROSConnection ros;
    private InputDevice hand;
    private float nextPublish;
    private bool lastMode;
    private bool released;
    private bool restoreGripper;
    private bool configured;
    private bool calibrationBlocked;
    private readonly string clockSession = System.Guid.NewGuid().ToString("N");

    [System.Serializable]
    private class ClockExchange
    {
        public string id;
        public long sent_ns;
        public long received_ns;
        public long replied_ns;
        public string session;
    }

    private static long UtcNanoseconds()
    {
        return (System.DateTime.UtcNow.Ticks - 621355968000000000L) * 100L;
    }

    private void OnClockPing(StringMsg ping)
    {
        long received = UtcNanoseconds();
        if (!isActiveAndEnabled) return;
        try
        {
            var reply = JsonUtility.FromJson<ClockExchange>(ping.data);
            if (reply == null || string.IsNullOrEmpty(reply.id) || reply.sent_ns <= 0) return;
            reply.received_ns = received;
            reply.session = clockSession;
            reply.replied_ns = UtcNanoseconds();
            ros.Publish("/vr/clock_pong", new StringMsg(JsonUtility.ToJson(reply)));
        }
        catch (System.ArgumentException exception)
        {
            Debug.LogWarning("[Clock sync] Invalid ping: " + exception.Message, this);
        }
    }

    public void SetCalibrationBlocked(bool blocked)
    {
        calibrationBlocked = blocked;
        released = false;
        nextPublish = 0f;
        if (ros != null) ros.Publish("/vr/right_hand/active", new BoolMsg(false));
        if (legacyPublisher != null) legacyPublisher.SetCalibrationBlocked(blocked);
    }

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>("/vr/right_hand/pose");
        ros.RegisterPublisher<BoolMsg>("/vr/right_hand/active");
        ros.RegisterPublisher<StringMsg>("/vr/clock_pong");
        ros.Subscribe<StringMsg>("/teleop/clock_ping", OnClockPing);
        if (legacyPublisher == null) legacyPublisher = GetComponent<ControllerTwistPublisher>();
        if (legacyPublisher != null) restoreGripper = legacyPublisher.publishGripperCommands;
        ApplyMode();
    }

    private void ApplyMode()
    {
        if (legacyPublisher != null)
        {
            // Retain the existing gripper publisher and simulated pickup state.
            legacyPublisher.poseInputMode = poseMode;
            legacyPublisher.publishGripperCommands = restoreGripper;
        }
        released = false;
        ros.Publish("/vr/right_hand/active", new BoolMsg(false));
        lastMode = poseMode;
        configured = true;
    }

    private void Update()
    {
        if (ros == null) return;
        if (poseMode != lastMode) ApplyMode();
        if (!poseMode || Time.unscaledTime < nextPublish) return;
        nextPublish = Time.unscaledTime + 1f / Mathf.Max(1f, publishHz);
        if (calibrationBlocked)
        {
            released = false;
            ros.Publish("/vr/right_hand/active", new BoolMsg(false));
            return;
        }
        hand = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        bool valid = hand.isValid && hand.TryGetFeatureValue(CommonUsages.isTracked, out bool tracked) && tracked
            && rightController != null && robotBase != null;
        bool pressed = false;
        if (valid)
        {
            if (hand.TryGetFeatureValue(CommonUsages.trigger, out float trigger)) pressed = trigger > 0.5f;
            else if (hand.TryGetFeatureValue(CommonUsages.triggerButton, out bool button)) pressed = button;
        }
        if (!valid) released = false;
        else if (!pressed) released = true;
        ros.Publish("/vr/right_hand/active", new BoolMsg(valid && pressed && released));
        if (!valid) return;
        var msg = new PoseStampedMsg();
        msg.header.frame_id = frameId;
        long ns = UtcNanoseconds();
        msg.header.stamp.sec = (int)(ns / 1000000000L);
        msg.header.stamp.nanosec = (uint)(ns % 1000000000L);
        // A fixed visual base reference supplies axes only; ROS anchors to actual TCP.
        msg.pose.position = robotBase.InverseTransformPoint(rightController.position).To<FLU>();
        msg.pose.orientation = (Quaternion.Inverse(robotBase.rotation) * rightController.rotation).To<FLU>();
        ros.Publish("/vr/right_hand/pose", msg);
    }

    private void OnDisable()
    {
        if (ros != null) ros.Publish("/vr/right_hand/active", new BoolMsg(false));
        if (configured && legacyPublisher != null) legacyPublisher.poseInputMode = false;
        released = false;
    }

    private void OnDestroy()
    {
        if (ros != null) ros.Unsubscribe("/teleop/clock_ping");
    }
}
