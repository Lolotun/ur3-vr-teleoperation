using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Keeps all gripper finger links frozen in their startup pose relative to robotiq_arg2f_base_link.
/// Disables ArticulationBody/Rigidbody physics on the gripper subtree so fingers do not jiggle.
/// </summary>
public class Ur3GripperRigidizer : MonoBehaviour
{
    public Transform gripperRoot;

    private readonly List<CachedPose> _cachedPoses = new();

    private struct CachedPose
    {
        public Transform Transform;
        public Vector3 LocalPosition;
        public Quaternion LocalRotation;
    }

    private void Awake()
    {
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
            Debug.LogWarning("[Ur3GripperRigidizer] gripperRoot is not assigned.", this);
            return;
        }

        CacheStartupPoses();
        DisableGripperPhysics();
    }

    private void LateUpdate()
    {
        RestoreStartupPoses();
    }

    private void CacheStartupPoses()
    {
        _cachedPoses.Clear();
        foreach (Transform child in gripperRoot.GetComponentsInChildren<Transform>(true))
        {
            if (child == gripperRoot)
            {
                continue;
            }

            _cachedPoses.Add(new CachedPose
            {
                Transform = child,
                LocalPosition = child.localPosition,
                LocalRotation = child.localRotation,
            });
        }
    }

    private void DisableGripperPhysics()
    {
        foreach (ArticulationBody body in gripperRoot.GetComponentsInChildren<ArticulationBody>(true))
        {
            body.useGravity = false;
            body.enabled = false;
        }

        foreach (Rigidbody body in gripperRoot.GetComponentsInChildren<Rigidbody>(true))
        {
            body.isKinematic = true;
            body.useGravity = false;
        }
    }

    private void RestoreStartupPoses()
    {
        foreach (CachedPose pose in _cachedPoses)
        {
            if (pose.Transform == null)
            {
                continue;
            }

            pose.Transform.localPosition = pose.LocalPosition;
            pose.Transform.localRotation = pose.LocalRotation;
        }
    }
}
