using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.XR;

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
    [Tooltip("Per-axis sign correction in ROS base_link (FLU) after Unity conversion. Use +/-1 to flip roll/pitch/yaw if a rotation feels inverted.")]
    public Vector3 angularRosSigns = Vector3.one;
    [Header("Robotiq Gripper")]
    public bool publishGripperCommands = true;
    public string gripperTopic = "/team_1/gripper_command";
    public float gripCloseThreshold = 0.65f;
    public bool animateGripperVisual = true;
    public Transform gripperRoot;
    public bool useGripButtonForGripper = true;
    public float gripperVisualCloseDistance = 0.012f;
    public float gripperVisualSpeed = 10f;

    private InputDevice rightHandDevice;
    private ROSConnection ros;
    private TwistStampedMsg twistMsg;
    private StringMsg gripperMsg;
    private float publishInterval;
    private float nextPublishTime;
    private bool wasTriggerPressed;
    private bool wasGripClosed;
    private bool hasPublishedGripState;
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
        PublishGripperCommandIfChanged();
        UpdateGripperVisual();

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
        if (!publishGripperCommands || ros == null)
        {
            return;
        }

        bool gripClosed = IsRightGripClosed();
        if (hasPublishedGripState && gripClosed == wasGripClosed)
        {
            return;
        }

        gripperMsg.data = gripClosed ? "close" : "open";
        ros.Publish(gripperTopic, gripperMsg);
        wasGripClosed = gripClosed;
        hasPublishedGripState = true;
        Debug.Log($"[ControllerTwistPublisher] gripper {gripperMsg.data}");
    }

    private bool IsRightGripClosed()
    {
        if (!rightHandDevice.isValid)
        {
            rightHandDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        }

        if (!rightHandDevice.isValid)
        {
            return false;
        }

        if (rightHandDevice.TryGetFeatureValue(CommonUsages.grip, out float gripValue))
        {
            return gripValue > gripCloseThreshold;
        }

        if (rightHandDevice.TryGetFeatureValue(CommonUsages.gripButton, out bool gripButton))
        {
            return gripButton;
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

        float closeDistance = wasGripClosed ? gripperVisualCloseDistance : 0f;
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
