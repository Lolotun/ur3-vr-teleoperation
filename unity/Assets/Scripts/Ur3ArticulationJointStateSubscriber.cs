using System;
using System.Collections.Generic;
using System.Text;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Drives the six UR3 arm joints from sensor_msgs/JointState.
///
/// Calibration: enable Debug Mode, select one joint, then command 0 -> +10 -> 0 degrees.
/// If its physical direction is wrong, change only that joint's Sign to -1.
/// </summary>
public class Ur3ArticulationJointStateSubscriber : MonoBehaviour
{
    private const string ShoulderPan = "shoulder_pan_joint";
    private const string ShoulderLift = "shoulder_lift_joint";
    private const string Elbow = "elbow_joint";
    private const string Wrist1 = "wrist_1_joint";
    private const string Wrist2 = "wrist_2_joint";
    private const string Wrist3 = "wrist_3_joint";

    private static readonly string[] JointNames =
    {
        ShoulderPan, ShoulderLift, Elbow, Wrist1, Wrist2, Wrist3,
    };

    private static readonly Dictionary<string, string> LinkNames = new()
    {
        { ShoulderPan, "shoulder_link" },
        { ShoulderLift, "upper_arm_link" },
        { Elbow, "forearm_link" },
        { Wrist1, "wrist_1_link" },
        { Wrist2, "wrist_2_link" },
        { Wrist3, "wrist_3_link" },
    };

    public enum DebugJoint
    {
        ShoulderPan,
        ShoulderLift,
        Elbow,
        Wrist1,
        Wrist2,
        Wrist3,
    }

    [Serializable]
    public class JointSign
    {
        public string jointName;
        [Tooltip("Use +1 normally. Set only this value to -1 when the calibration motion is reversed.")]
        [Range(-1, 1)] public int sign = 1;
    }

    [Header("ROS")]
    public string topic = "/joint_states";
    public Transform robotRoot;

    [Header("Articulation drive")]
    [Min(0f)] public float driveStiffness = 10000f;
    [Min(0f)] public float driveDamping = 1000f;
    [Min(0f)] public float driveForceLimit = 1000f;
    [Tooltip("Logs one raw ROS-to-xDrive table for every received /joint_states message. Enable only during the equivalence test.")]
    public bool logJointStateTable;
    public JointSign[] jointSigns =
    {
        new() { jointName = ShoulderPan },
        new() { jointName = ShoulderLift },
        new() { jointName = Elbow },
        new() { jointName = Wrist1 },
        new() { jointName = Wrist2 },
        new() { jointName = Wrist3 },
    };

    [Header("Temporary one-joint calibration")]
    public bool debugMode;
    public DebugJoint debugJoint;
    [Range(-180f, 180f)] public float debugAngleDegrees;

    private readonly Dictionary<string, ArticulationBody> _joints = new();
    private readonly Dictionary<string, float> _positionsRadians = new();
    private readonly object _stateLock = new();
    private bool _hasNewJointState;
    private Transform _gripperRoot;

    private void Awake()
    {
        if (robotRoot == null)
            robotRoot = transform;

        GameObject gripperObject = GameObject.Find("robotiq_arg2f_base_link");
        if (gripperObject != null)
            _gripperRoot = gripperObject.transform;

        var transforms = robotRoot.GetComponentsInChildren<Transform>(true);
        foreach (var jointName in JointNames)
        {
            var linkName = LinkNames[jointName];
            foreach (var candidate in transforms)
            {
                if (candidate.name != linkName)
                    continue;

                var body = candidate.GetComponent<ArticulationBody>();
                if (body != null)
                    _joints[jointName] = body;
                break;
            }

            if (!_joints.ContainsKey(jointName))
                Debug.LogError($"{nameof(Ur3ArticulationJointStateSubscriber)}: no ArticulationBody found for {jointName}.", this);
        }

        ConfigureArticulationBodies(transforms);
        foreach (var body in _joints.Values)
            ConfigureArmJoint(body);

        var fallback = GetComponent<Ur3JointStateSubscriber>();
        if (fallback != null && fallback.enabled)
            Debug.LogError("Both UR3 joint subscribers are enabled. Disable Ur3JointStateSubscriber before running the Articulation subscriber.", this);
    }

    private void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(topic, OnJointState);
    }

    private void OnJointState(JointStateMsg message)
    {
        if (debugMode)
            return;

        var count = Mathf.Min(message.name.Length, message.position.Length);
        lock (_stateLock)
        {
            for (var i = 0; i < count; i++)
            {
                if (_joints.ContainsKey(message.name[i]))
                    _positionsRadians[message.name[i]] = (float)message.position[i];
            }

            _hasNewJointState = true;
        }
    }

    private void FixedUpdate()
    {
        if (debugMode)
        {
            var jointName = JointNames[(int)debugJoint];
            if (_joints.TryGetValue(jointName, out var debugBody))
                SetTargetDegrees(jointName, debugBody, debugAngleDegrees);
            return;
        }

        Dictionary<string, float> rawPositions;
        bool receivedNewState;
        lock (_stateLock)
        {
            rawPositions = new Dictionary<string, float>(_positionsRadians);
            receivedNewState = _hasNewJointState;
            _hasNewJointState = false;
        }

        foreach (var position in rawPositions)
        {
            if (_joints.TryGetValue(position.Key, out var body))
                SetTargetDegrees(position.Key, body, position.Value * Mathf.Rad2Deg);
        }

        if (logJointStateTable && receivedNewState)
            LogJointStateTable(rawPositions);
    }

    private void ConfigureArticulationBodies(Transform[] transforms)
    {
        foreach (var candidate in transforms)
        {
            var body = candidate.GetComponent<ArticulationBody>();
            if (body == null)
                continue;

            if (IsGripperArticulation(candidate))
            {
                body.useGravity = false;
                body.enabled = false;
                continue;
            }

            body.useGravity = false;
            body.enabled = true;
        }

        foreach (var candidate in transforms)
        {
            if (candidate.name != "base_link")
                continue;

            var baseBody = candidate.GetComponent<ArticulationBody>();
            if (baseBody == null)
            {
                Debug.LogError($"{nameof(Ur3ArticulationJointStateSubscriber)}: base_link has no ArticulationBody.", this);
                return;
            }

            baseBody.immovable = true;
            baseBody.useGravity = false;
            baseBody.enabled = true;
            return;
        }

        Debug.LogError($"{nameof(Ur3ArticulationJointStateSubscriber)}: base_link was not found below robotRoot.", this);
    }

    private bool IsGripperArticulation(Transform candidate)
    {
        return _gripperRoot != null && candidate != _gripperRoot && candidate.IsChildOf(_gripperRoot);
    }

    private void ConfigureArmJoint(ArticulationBody body)
    {
        body.useGravity = false;
        var drive = body.xDrive;
        drive.stiffness = driveStiffness;
        drive.damping = driveDamping;
        drive.forceLimit = driveForceLimit;
        body.xDrive = drive;
        body.enabled = true;
    }

    private void SetTargetDegrees(string jointName, ArticulationBody body, float degrees)
    {
        var drive = body.xDrive;
        drive.target = GetSign(jointName) * degrees;
        body.xDrive = drive;
    }

    private int GetSign(string jointName)
    {
        foreach (var entry in jointSigns)
        {
            if (entry != null && entry.jointName == jointName)
                return entry.sign < 0 ? -1 : 1;
        }

        return 1;
    }

    private void LogJointStateTable(IReadOnlyDictionary<string, float> rawPositions)
    {
        var table = new StringBuilder("[UR3 raw /joint_states -> Articulation xDrive]\n" +
                                      "joint name | ROS q rad | expected deg | actual xDrive.target | sign");
        foreach (var jointName in JointNames)
        {
            if (!rawPositions.TryGetValue(jointName, out var radians) ||
                !_joints.TryGetValue(jointName, out var body))
                continue;

            var degrees = radians * Mathf.Rad2Deg;
            table.Append($"\n{jointName} | {radians:F6} | {degrees:F3} | {body.xDrive.target:F3} | {GetSign(jointName)}");
        }

        Debug.Log(table.ToString(), this);
    }
}
