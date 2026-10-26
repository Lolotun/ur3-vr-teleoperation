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

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>("/vr/right_hand/pose");
        ros.RegisterPublisher<BoolMsg>("/vr/right_hand/active");
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
        var now = System.DateTimeOffset.UtcNow;
        var msg = new PoseStampedMsg();
        msg.header.frame_id = frameId;
        long ms = now.ToUnixTimeMilliseconds();
        msg.header.stamp.sec = (int)(ms / 1000);
        msg.header.stamp.nanosec = (uint)((ms % 1000) * 1000000);
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
}
