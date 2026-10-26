using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

/// <summary>
/// Read-only FK diagnostic: publishes Unity's imported tool0 pose relative to
/// Unity's imported base_link. It does not use the gripper/finger tip or alter
/// any transform, articulation drive, or ROS command.
/// </summary>
public class Ur3Tool0PosePublisher : MonoBehaviour
{
    [Header("ROS")]
    public string topic = "/team_1/unity_tool0";
    [Min(1f)] public float publishHz = 20f;
    public Transform robotRoot;

    private Transform _baseLink;
    private Transform _tool0;
    private ROSConnection _ros;
    private PoseStampedMsg _message;
    private float _nextPublishTime;

    private void Awake()
    {
        if (robotRoot == null)
            robotRoot = transform;

        foreach (var candidate in robotRoot.GetComponentsInChildren<Transform>(true))
        {
            if (candidate.name == "base_link")
                _baseLink = candidate;
            else if (candidate.name == "tool0")
                _tool0 = candidate;
        }

        if (_baseLink == null || _tool0 == null)
            Debug.LogError($"{nameof(Ur3Tool0PosePublisher)}: base_link or tool0 was not found below robotRoot.", this);
    }

    private void Start()
    {
        if (_baseLink == null || _tool0 == null)
        {
            enabled = false;
            return;
        }

        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<PoseStampedMsg>(topic);
        _message = new PoseStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg { frame_id = "base_link" },
            pose = new PoseMsg(),
        };
        Debug.Log($"[{nameof(Ur3Tool0PosePublisher)}] Publishing Unity tool0 relative to base_link on {topic}", this);
    }

    private void LateUpdate()
    {
        if (Time.time < _nextPublishTime)
            return;

        _nextPublishTime = Time.time + 1f / publishHz;
        // The URDF importer stores link coordinates in the Unity representation
        // of each URDF frame. First make tool0 local to base_link, then apply
        // the single Unity-to-FLU basis conversion to that relative pose.
        var relativePositionUnity = _baseLink.InverseTransformPoint(_tool0.position);
        var relativeRotationUnity = Quaternion.Inverse(_baseLink.rotation) * _tool0.rotation;

        _message.header.frame_id = "base_link";
        var now = Time.realtimeSinceStartupAsDouble;
        var seconds = Mathf.FloorToInt((float)now);
        _message.header.stamp = new TimeMsg(seconds, (uint)((now - seconds) * 1_000_000_000.0));
        _message.pose.position = relativePositionUnity.To<FLU>();
        _message.pose.orientation = relativeRotationUnity.To<FLU>();
        _ros.Publish(topic, _message);
    }
}
