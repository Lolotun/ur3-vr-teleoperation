using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Read-only diagnostic publisher. It reports the actual ArticulationBody joint
/// coordinates so they can be compared directly with incoming ROS /joint_states.
/// It never writes drives or transforms.
/// </summary>
public class Ur3ArticulationJointStatePublisher : MonoBehaviour
{
    private static readonly string[] JointNames =
    {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    };

    private static readonly string[] LinkNames =
    {
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
    };

    [Header("ROS")]
    public string topic = "/team_1/unity_joint_states";
    [Min(1f)] public float publishHz = 20f;
    public Transform robotRoot;

    private readonly ArticulationBody[] _joints = new ArticulationBody[6];
    private ROSConnection _ros;
    private JointStateMsg _message;
    private float _nextPublishTime;

    private void Awake()
    {
        if (robotRoot == null)
            robotRoot = transform;

        var transforms = robotRoot.GetComponentsInChildren<Transform>(true);
        for (var i = 0; i < LinkNames.Length; i++)
        {
            foreach (var candidate in transforms)
            {
                if (candidate.name != LinkNames[i])
                    continue;

                _joints[i] = candidate.GetComponent<ArticulationBody>();
                break;
            }

            if (_joints[i] == null)
                Debug.LogError($"{nameof(Ur3ArticulationJointStatePublisher)}: no ArticulationBody found for {JointNames[i]}.", this);
        }
    }

    private void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<JointStateMsg>(topic);
        _message = new JointStateMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg(),
            name = JointNames,
            position = new double[JointNames.Length],
        };
        Debug.Log($"[{nameof(Ur3ArticulationJointStatePublisher)}] Publishing actual joint positions on {topic}", this);
    }

    private void FixedUpdate()
    {
        if (Time.time < _nextPublishTime)
            return;

        _nextPublishTime = Time.time + 1f / publishHz;
        for (var i = 0; i < _joints.Length; i++)
        {
            if (_joints[i] == null || _joints[i].jointPosition.dofCount < 1)
                return;

            // ArticulationBody rotational jointPosition is expressed in radians.
            _message.position[i] = _joints[i].jointPosition[0];
        }

        var now = Time.realtimeSinceStartupAsDouble;
        var seconds = Mathf.FloorToInt((float)now);
        _message.header.stamp = new TimeMsg(seconds, (uint)((now - seconds) * 1_000_000_000.0));
        _ros.Publish(topic, _message);
    }
}
