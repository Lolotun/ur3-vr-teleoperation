using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TcpActualSubscriber : MonoBehaviour
{
    public string topic = "/team_1/tcp_actual";
    [Tooltip("Interpret incoming pose as coordinates relative to Base Frame, rather than Unity world coordinates.")]
    public bool poseIsRelativeToBase;
    [Tooltip("Unity Transform corresponding to the ROS frame_id when Pose Is Relative To Base is enabled.")]
    public Transform baseFrame;
    public Transform targetToSnapOnFirstPose;
    public bool snapTargetOnFirstPose = true;
    public float positionSmoothTime = 0.04f;
    public float rotationSmoothSpeed = 20f;

    private ROSConnection ros;
    private Vector3 targetPosition;
    private Quaternion targetRotation = Quaternion.identity;
    private Vector3 positionVelocity;
    private bool hasPose;
    private bool snappedTarget;

    private void Start()
    {
        targetPosition = transform.position;
        targetRotation = transform.rotation;

        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PoseStampedMsg>(topic, OnPose);
        Debug.Log($"[TcpActualSubscriber] Subscribed to {topic}");
    }

    private void Update()
    {
        if (!hasPose)
        {
            return;
        }

        transform.position = Vector3.SmoothDamp(
            transform.position,
            targetPosition,
            ref positionVelocity,
            Mathf.Max(0.001f, positionSmoothTime)
        );
        transform.rotation = Quaternion.Slerp(
            transform.rotation,
            targetRotation,
            1f - Mathf.Exp(-Mathf.Max(0f, rotationSmoothSpeed) * Time.deltaTime)
        );
    }

    private void OnPose(PoseStampedMsg msg)
    {
        Vector3 unityPosition = msg.pose.position.From<FLU>();
        Quaternion unityRotation = msg.pose.orientation.From<FLU>();
        if (poseIsRelativeToBase && baseFrame != null)
        {
            targetPosition = baseFrame.TransformPoint(unityPosition);
            targetRotation = baseFrame.rotation * unityRotation;
        }
        else
        {
            targetPosition = unityPosition;
            targetRotation = unityRotation;
        }
        hasPose = true;

        if (snapTargetOnFirstPose && !snappedTarget && targetToSnapOnFirstPose != null)
        {
            targetToSnapOnFirstPose.position = targetPosition;
            targetToSnapOnFirstPose.rotation = targetRotation;
            snappedTarget = true;
        }
    }
}
