using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

/// <summary>
/// Publishes TCP_Target pose to ROS (Day 1: Unity ↔ ROS connectivity).
/// </summary>
public class TcpTargetPublisher : MonoBehaviour
{
    [Header("ROS")]
    public string topic = "/team_1/tcp_target";
    public string frameId = "world";
    public float publishHz = 30f;
    public ClutchedTeleopTarget clutch;
    public bool publishOnlyWhileClutched = true;

    private ROSConnection ros;
    private PoseStampedMsg poseMsg;
    private float publishInterval;
    private float nextPublishTime;

    private void Start()
    {
        if (clutch == null)
        {
            clutch = GetComponent<ClutchedTeleopTarget>();
        }

        ros = ROSConnection.GetOrCreateInstance();
        publishInterval = publishHz > 0f ? 1f / publishHz : 0f;

        poseMsg = new PoseStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg { frame_id = frameId },
            pose = new PoseMsg(),
        };

        ros.RegisterPublisher<PoseStampedMsg>(topic);
    }

    private void Update()
    {
        if (publishOnlyWhileClutched && clutch != null && !clutch.IsClutchActive)
        {
            return;
        }

        if (publishInterval <= 0f || Time.time < nextPublishTime)
        {
            return;
        }

        nextPublishTime = Time.time + publishInterval;
        PublishCurrentPose();
    }

    private void PublishCurrentPose()
    {
        poseMsg.header.frame_id = frameId;
        poseMsg.pose.position = transform.position.To<FLU>();
        poseMsg.pose.orientation = transform.rotation.To<FLU>();

        ros.Publish(topic, poseMsg);
    }
}
