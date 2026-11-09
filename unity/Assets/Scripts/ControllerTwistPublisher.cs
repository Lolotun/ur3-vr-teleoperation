using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.XR;
using System.Globalization;

public class ControllerTwistPublisher : MonoBehaviour
{
    public Transform rightController;
    [Tooltip("Unity Transform of the visual URDF base_link. The published velocity is expressed in this frame.")]
    public Transform robotBase;
    public string topic = "/team_1/twist_command";
    public string frameId = "base_link";
    public float publishHz = 30f;
    public float deadZoneMeters = 0.015f;
    public float deadZoneDegrees = 1.0f;
    public float linearGain = 0.7f;
    public float angularGain = 1.0f;
    public float maxLinearSpeed = 0.12f;
    public float maxAngularSpeedRad = 0.2f;
    public bool logAngularDebug;
    [Header("Mode")]
    [Tooltip("Simulation: publish controller rotation as angular velocity for MoveIt Servo. Real robot: keep linear-only teleop (angular is zeroed on ROS side too).")]
    public bool publishAngularVelocity = true;
    [HideInInspector] public bool poseInputMode;
    [Tooltip("Per-axis sign correction in ROS base_link (FLU) after Unity conversion. Use +/-1 to flip roll/pitch/yaw if a rotation feels inverted.")]
    public Vector3 angularRosSigns = Vector3.one;
    [Header("Robotiq Gripper")]
    public bool publishGripperCommands = true;
    public string gripperTopic = "/team_1/gripper_command";
    [Tooltip("Send proportional Grip position instead of binary open/close.")]
    public bool analogGripperControl = true;
    [Tooltip("Right A toggles between holding the current position and following Grip.")]
    public bool latchGripperWithA = true;
    public float gripperPublishHz = 10f;
    [Range(0f, 0.2f)] public float gripperDeadZone = 0.02f;
    public float gripCloseThreshold = 0.65f;
    public bool animateGripperVisual = true;
    public Transform gripperRoot;
    public bool useGripButtonForGripper = true;
    public float gripperVisualCloseDistance = 0.012f;
    public float gripperVisualSpeed = 10f;
    [Header("Dataset recording")]
    [Tooltip("Right B sends a recording toggle event to vr-dagger.")]
    public bool publishDatasetButton = true;
    private const string DatasetToggleTopic = "/vr/dataset/toggle";
    private const string DatasetStatusTopic = "/dataset/recording_status";
    private readonly ButtonPressEdge datasetButton = new ButtonPressEdge();
    private readonly string datasetSession = System.Guid.NewGuid().ToString("N");
    private long datasetSequence;
    private string lastDatasetStatus;

    [System.Serializable]
    private class DatasetToggle
    {
        public int schema = 1;
        public string session;
        public long sequence;
        public long stamp_ns;
    }

    [System.Serializable]
    private class DatasetStatus
    {
        public string state = "";
        public string episode = "";
        public bool ready = false;
        public string error = "";
    }

    private void OnDatasetStatus(StringMsg status)
    {
        try
        {
            var value = JsonUtility.FromJson<DatasetStatus>(status.data);
            if (value == null) return;
            string summary = $"{value.state} {value.episode} ready={value.ready} {value.error}";
            if (summary == lastDatasetStatus) return;
            lastDatasetStatus = summary;
            Debug.Log("[Dataset] " + summary, this);
        }
        catch (System.ArgumentException exception)
        {
            Debug.LogWarning("[Dataset] Invalid status: " + exception.Message, this);
        }
    }

    private InputDevice rightHandDevice;
    private ROSConnection ros;
    private TwistStampedMsg twistMsg;
    private StringMsg gripperMsg;
    private float publishInterval;
    private float nextPublishTime;
    private bool wasTriggerPressed;
    private bool hasPublishedGripState;
    private int lastGripperPosition;
    private float nextGripperPublish;
    private readonly GripperCommandState gripperState = new GripperCommandState();
    public float GripperPosition => gripperState.Position;
    public bool GripperLatched => gripperState.Latched;
    private Transform rightOuterKnuckle;
    private Transform leftOuterKnuckle;
    private Transform rightInnerKnuckle;
    private Transform leftInnerKnuckle;
    private Vector3 rightOuterKnuckleOpenPosition;
    private Vector3 leftOuterKnuckleOpenPosition;
    private Vector3 rightInnerKnuckleOpenPosition;
    private Vector3 leftInnerKnuckleOpenPosition;
    private bool hasNeutralPose;
    private Vector3 neutralControllerPosition;
    private Quaternion neutralControllerRotation;
    private bool calibrationBlocked;

    public void SetCalibrationBlocked(bool blocked)
    {
        calibrationBlocked = blocked;
        gripperState.DisarmPrimary();
        if (ros != null && twistMsg != null) PublishZeroTwist();
        wasTriggerPressed = false;
        hasNeutralPose = false;
    }

    private void Start()
    {
        if (robotBase == null)
        {
            GameObject baseObject = GameObject.Find("base_link");
            if (baseObject != null)
            {
                robotBase = baseObject.transform;
                Debug.Log("[ControllerTwistPublisher] robotBase was auto-assigned to base_link.");
            }
            else
            {
                Debug.LogWarning(
                    "[ControllerTwistPublisher] robotBase is not assigned. " +
                    "Twist values will only be correct if the Unity world axes already match base_link. " +
                    "Assign the URDF base_link Transform before using this with a robot."
                );
            }
        }

        ros = ROSConnection.GetOrCreateInstance();
        publishInterval = publishHz > 0f ? 1f / publishHz : 0f;
        twistMsg = new TwistStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg { frame_id = frameId },
            twist = new TwistMsg(),
        };

        ros.RegisterPublisher<TwistStampedMsg>(topic);
        ros.RegisterPublisher<StringMsg>(DatasetToggleTopic);
        ros.Subscribe<StringMsg>(DatasetStatusTopic, OnDatasetStatus);
        if (publishGripperCommands)
        {
            gripperMsg = new StringMsg();
            ros.RegisterPublisher<StringMsg>(gripperTopic);
        }

        SetupGripperVisual();
        Debug.Log($"[ControllerTwistPublisher] Publishing {topic} @ {publishHz:F1} Hz");
    }

    private void Update()
    {
        PublishDatasetButton();
        if (calibrationBlocked) return;
        PublishGripperCommandIfChanged();
        UpdateGripperVisual();

        if (poseInputMode)
        {
            if (wasTriggerPressed) PublishZeroTwist();
            wasTriggerPressed = false;
            hasNeutralPose = false;
            return;
        }

        if (rightController == null || publishInterval <= 0f || Time.time < nextPublishTime)
        {
            return;
        }

        nextPublishTime = Time.time + publishInterval;

        bool triggerPressed = IsRightTriggerPressed();
        if (triggerPressed && !wasTriggerPressed)
        {
            neutralControllerPosition = rightController.position;
            neutralControllerRotation = rightController.rotation;
            hasNeutralPose = true;
        }

        if (triggerPressed && hasNeutralPose)
        {
            PublishTwistFromControllerOffset();
        }
        else if (wasTriggerPressed)
        {
            PublishZeroTwist();
            hasNeutralPose = false;
        }

        wasTriggerPressed = triggerPressed;
    }

    private void PublishDatasetButton()
    {
        if (!rightHandDevice.isValid)
            rightHandDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        bool pressed = false;
        bool available = publishDatasetButton && ros != null
            && ros.HasConnectionThread && !ros.HasConnectionError && rightHandDevice.isValid
            && rightHandDevice.TryGetFeatureValue(CommonUsages.isTracked, out bool tracked) && tracked
            && rightHandDevice.TryGetFeatureValue(CommonUsages.secondaryButton, out pressed);
        if (!datasetButton.Update(available, pressed)) return;
        var message = new DatasetToggle
        {
            session = datasetSession,
            sequence = ++datasetSequence,
            stamp_ns = (System.DateTime.UtcNow.Ticks - 621355968000000000L) * 100L,
        };
        ros.Publish(DatasetToggleTopic, new StringMsg(JsonUtility.ToJson(message)));
        Debug.Log("[Dataset] Right B: recording toggle sent", this);
    }

    private void OnDisable()
    {
        datasetButton.Update(false, false);
    }

    private void OnDestroy()
    {
        if (ros != null) ros.Unsubscribe(DatasetStatusTopic);
    }

    private bool IsRightTriggerPressed()
    {
        if (!rightHandDevice.isValid)
        {
            rightHandDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        }

        if (!rightHandDevice.isValid)
        {
            return false;
        }

        if (rightHandDevice.TryGetFeatureValue(CommonUsages.trigger, out float triggerValue))
        {
            return triggerValue > 0.5f;
        }

        if (rightHandDevice.TryGetFeatureValue(CommonUsages.triggerButton, out bool triggerButton))
        {
            return triggerButton;
        }

        return false;
    }

    private void PublishGripperCommandIfChanged()
    {
        if (ros == null)
        {
            return;
        }

        if (!TryReadRightGrip(out float grip, out bool primaryPressed))
        {
            // Tracking loss must not toggle the latch or send an open command.
            gripperState.DisarmPrimary();
            return;
        }
        grip = grip < gripperDeadZone ? 0f : Mathf.Clamp01(grip);
        if (!analogGripperControl) grip = grip > gripCloseThreshold ? 1f : 0f;
        bool wasLatched = gripperState.Latched;
        gripperState.Update(grip, latchGripperWithA && primaryPressed);
        if (wasLatched != gripperState.Latched)
        {
            hasPublishedGripState = false;
            Debug.Log($"[ControllerTwistPublisher] gripper {(gripperState.Latched ? "latched" : "following Grip")} at {gripperState.Position:F3}");
        }

        if (!publishGripperCommands || Time.unscaledTime < nextGripperPublish) return;
        int position = Mathf.RoundToInt(gripperState.Position * 255f);
        if (hasPublishedGripState && position == lastGripperPosition) return;
        if (gripperMsg == null)
        {
            gripperMsg = new StringMsg();
            ros.RegisterPublisher<StringMsg>(gripperTopic);
        }
        // Preserve the existing String topic; normalize to 0..1 with a locale-independent decimal point.
        gripperMsg.data = (position / 255f).ToString("F3", CultureInfo.InvariantCulture);
        ros.Publish(gripperTopic, gripperMsg);
        lastGripperPosition = position;
        nextGripperPublish = Time.unscaledTime + 1f / Mathf.Max(1f, gripperPublishHz);
        hasPublishedGripState = true;
    }

    private bool TryReadRightGrip(out float grip, out bool primaryPressed)
    {
        grip = 0f;
        primaryPressed = false;
        if (!rightHandDevice.isValid)
        {
            rightHandDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        }

        if (!rightHandDevice.isValid)
        {
            return false;
        }
        if (!rightHandDevice.TryGetFeatureValue(CommonUsages.isTracked, out bool tracked) || !tracked)
            return false;
        rightHandDevice.TryGetFeatureValue(CommonUsages.primaryButton, out primaryPressed);

        if (rightHandDevice.TryGetFeatureValue(CommonUsages.grip, out float gripValue))
        {
            if (float.IsNaN(gripValue) || float.IsInfinity(gripValue)) return false;
            grip = gripValue;
            return true;
        }

        if (rightHandDevice.TryGetFeatureValue(CommonUsages.gripButton, out bool gripButton))
        {
            grip = gripButton ? 1f : 0f;
            return true;
        }

        return false;
    }

    private void SetupGripperVisual()
    {
        if (!animateGripperVisual)
        {
            return;
        }

        if (gripperRoot == null)
        {
            GameObject gripperObject = GameObject.Find("robotiq_arg2f_base_link");
            if (gripperObject != null)
            {
                gripperRoot = gripperObject.transform;
            }
        }

        if (gripperRoot == null)
        {
            Debug.LogWarning("[ControllerTwistPublisher] gripperRoot is not assigned; visual gripper animation disabled.");
            return;
        }

        rightOuterKnuckle = FindChildRecursive(gripperRoot, "right_outer_knuckle");
        leftOuterKnuckle = FindChildRecursive(gripperRoot, "left_outer_knuckle");
        rightInnerKnuckle = FindChildRecursive(gripperRoot, "right_inner_knuckle");
        leftInnerKnuckle = FindChildRecursive(gripperRoot, "left_inner_knuckle");

        CacheOpenPosition(rightOuterKnuckle, ref rightOuterKnuckleOpenPosition);
        CacheOpenPosition(leftOuterKnuckle, ref leftOuterKnuckleOpenPosition);
        CacheOpenPosition(rightInnerKnuckle, ref rightInnerKnuckleOpenPosition);
        CacheOpenPosition(leftInnerKnuckle, ref leftInnerKnuckleOpenPosition);
    }

    private void CacheOpenPosition(Transform joint, ref Vector3 position)
    {
        if (joint != null)
        {
            position = joint.localPosition;
        }
    }

    private Transform FindChildRecursive(Transform root, string childName)
    {
        foreach (Transform child in root)
        {
            if (child.name == childName)
            {
                return child;
            }

            Transform match = FindChildRecursive(child, childName);
            if (match != null)
            {
                return match;
            }
        }

        return null;
    }

    private void UpdateGripperVisual()
    {
        if (!animateGripperVisual || gripperRoot == null)
        {
            return;
        }

        float closeDistance = gripperState.Position * gripperVisualCloseDistance;
        SetFingerGroupVisual(rightOuterKnuckle, rightOuterKnuckleOpenPosition, closeDistance);
        SetFingerGroupVisual(leftOuterKnuckle, leftOuterKnuckleOpenPosition, closeDistance);
        SetFingerGroupVisual(rightInnerKnuckle, rightInnerKnuckleOpenPosition, closeDistance);
        SetFingerGroupVisual(leftInnerKnuckle, leftInnerKnuckleOpenPosition, closeDistance);
    }

    private void SetFingerGroupVisual(Transform fingerRoot, Vector3 openPosition, float closeDistance)
    {
        if (fingerRoot == null)
        {
            return;
        }

        float direction = openPosition.x < 0f ? 1f : -1f;
        Vector3 targetPosition = openPosition + new Vector3(direction * closeDistance, 0f, 0f);
        fingerRoot.localPosition = Vector3.Lerp(
            fingerRoot.localPosition,
            targetPosition,
            1f - Mathf.Exp(-gripperVisualSpeed * Time.deltaTime)
        );
    }

    private void PublishTwistFromControllerOffset()
    {
        Vector3 offset = rightController.position - neutralControllerPosition;
        Quaternion rotationOffset = rightController.rotation * Quaternion.Inverse(neutralControllerRotation);
        rotationOffset.ToAngleAxis(out float angleDegrees, out Vector3 rotationAxis);
        if (angleDegrees > 180f)
        {
            angleDegrees -= 360f;
        }

        bool linearDead = offset.magnitude < deadZoneMeters;
        bool angularDead = Mathf.Abs(angleDegrees) < deadZoneDegrees;
        if (linearDead && angularDead)
        {
            PublishZeroTwist();
            return;
        }

        // The controller offset is deliberately measured in Unity world space:
        // moving the hand in one physical/world direction always requests the
        // same motion, irrespective of the current TCP orientation. MoveIt
        // Servo, however, is told that the Twist is in `frameId` (base_link),
        // so first express that world vector in the Unity representation of
        // base_link. Without this step the value and frame_id disagree whenever
        // the robot root is rotated in the scene.
        Vector3 worldVelocity = linearDead ? Vector3.zero : offset * linearGain;
        worldVelocity = Vector3.ClampMagnitude(worldVelocity, maxLinearSpeed);
        Vector3 worldAngularVelocity = Vector3.zero;
        if (publishAngularVelocity && !angularDead && rotationAxis.sqrMagnitude > 1e-6f)
        {
            worldAngularVelocity = rotationAxis.normalized * (angleDegrees * Mathf.Deg2Rad * angularGain);
            worldAngularVelocity = Vector3.ClampMagnitude(worldAngularVelocity, maxAngularSpeedRad);
        }

        Vector3 velocityInUnityBase = robotBase != null
            ? robotBase.InverseTransformDirection(worldVelocity)
            : worldVelocity;
        Vector3 angularVelocityInUnityBase = robotBase != null
            ? robotBase.InverseTransformDirection(worldAngularVelocity)
            : worldAngularVelocity;
        Vector3Msg rosVelocity = velocityInUnityBase.To<FLU>();
        Vector3Msg rosAngularVelocity = angularVelocityInUnityBase.To<FLU>();
        rosAngularVelocity = new Vector3Msg(
            rosAngularVelocity.x * angularRosSigns.x,
            rosAngularVelocity.y * angularRosSigns.y,
            rosAngularVelocity.z * angularRosSigns.z
        );

        UpdateHeader();
        twistMsg.twist.linear = rosVelocity;
        twistMsg.twist.angular = rosAngularVelocity;
        ros.Publish(topic, twistMsg);

        if (logAngularDebug && !angularDead)
        {
            Debug.Log(
                "[ControllerTwistPublisher] angular " +
                $"angle={angleDegrees:F1}deg axis=({rotationAxis.x:F2},{rotationAxis.y:F2},{rotationAxis.z:F2}) " +
                $"unityBase=({angularVelocityInUnityBase.x:F3}, {angularVelocityInUnityBase.y:F3}, {angularVelocityInUnityBase.z:F3}) " +
                $"ros=({rosAngularVelocity.x:F3}, {rosAngularVelocity.y:F3}, {rosAngularVelocity.z:F3})"
            );
        }
    }

    private void PublishZeroTwist()
    {
        UpdateHeader();
        twistMsg.twist.linear = new Vector3Msg();
        twistMsg.twist.angular = new Vector3Msg();
        ros.Publish(topic, twistMsg);
    }

    private void UpdateHeader()
    {
        double now = Time.realtimeSinceStartupAsDouble;
        int sec = Mathf.FloorToInt((float)now);
        uint nanosec = (uint)((now - sec) * 1_000_000_000.0);

        twistMsg.header.frame_id = frameId;
        twistMsg.header.stamp = new TimeMsg(sec, nanosec);
    }
}
