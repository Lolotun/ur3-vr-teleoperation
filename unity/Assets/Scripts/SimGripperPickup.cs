using UnityEngine;
using UnityEngine.XR;

/// <summary>
/// Simulation-only pickup: when the right grip is closed near a SimPickableObject, parent it to the gripper.
/// </summary>
public class SimGripperPickup : MonoBehaviour
{
    public Transform gripperRoot;
    public float attachRadius = 0.08f;
    public float gripCloseThreshold = 0.65f;
    public bool enabledInSimulation = true;
    public bool logDebug = true;

    private InputDevice rightHandDevice;
    private bool wasGripClosed;
    private Transform attachedObject;
    private Transform attachedOriginalParent;
    private SimPickableObject[] pickables;

    private void Start()
    {
        if (gripperRoot == null)
        {
            GameObject gripperObject = GameObject.Find("robotiq_arg2f_base_link");
            if (gripperObject != null)
            {
                gripperRoot = gripperObject.transform;
            }
        }

        pickables = FindObjectsByType<SimPickableObject>(FindObjectsSortMode.None);
        if (logDebug)
        {
            Debug.Log($"[SimGripperPickup] ready. pickables={pickables.Length}, radius={attachRadius:F2} m");
        }
    }

    private void Update()
    {
        if (!enabledInSimulation || gripperRoot == null)
        {
            return;
        }

        bool gripClosed = IsRightGripClosed();
        if (gripClosed && !wasGripClosed)
        {
            TryAttachNearestPickable();
        }
        else if (!gripClosed && wasGripClosed)
        {
            ReleaseAttachedObject();
        }

        wasGripClosed = gripClosed;
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

    private void TryAttachNearestPickable()
    {
        if (attachedObject != null || pickables == null)
        {
            return;
        }

        SimPickableObject bestPickable = null;
        float bestDistance = attachRadius;
        foreach (SimPickableObject pickable in pickables)
        {
            if (pickable == null)
            {
                continue;
            }

            Transform attachPoint = pickable.attachPoint != null ? pickable.attachPoint : pickable.transform;
            float distance = Vector3.Distance(gripperRoot.position, attachPoint.position);
            if (distance > bestDistance)
            {
                continue;
            }

            bestDistance = distance;
            bestPickable = pickable;
        }

        if (bestPickable == null)
        {
            if (logDebug)
            {
                float nearest = float.PositiveInfinity;
                string nearestName = "none";
                foreach (SimPickableObject pickable in pickables)
                {
                    if (pickable == null)
                    {
                        continue;
                    }

                    Transform attachPoint = pickable.attachPoint != null ? pickable.attachPoint : pickable.transform;
                    float distance = Vector3.Distance(gripperRoot.position, attachPoint.position);
                    if (distance < nearest)
                    {
                        nearest = distance;
                        nearestName = pickable.name;
                    }
                }

                Debug.Log(
                    $"[SimGripperPickup] grip closed but nothing in range. " +
                    $"nearest={nearestName} at {nearest:F3} m, need <= {attachRadius:F2} m"
                );
            }

            return;
        }

        attachedObject = bestPickable.attachPoint != null ? bestPickable.attachPoint : bestPickable.transform;
        attachedOriginalParent = attachedObject.parent;
        attachedObject.SetParent(gripperRoot, true);

        Rigidbody body = attachedObject.GetComponent<Rigidbody>();
        if (body != null)
        {
            body.isKinematic = true;
            body.useGravity = false;
        }

        Debug.Log($"[SimGripperPickup] attached {attachedObject.name} at distance {bestDistance:F3} m");
    }

    private void ReleaseAttachedObject()
    {
        if (attachedObject == null)
        {
            return;
        }

        attachedObject.SetParent(attachedOriginalParent, true);

        Rigidbody body = attachedObject.GetComponent<Rigidbody>();
        if (body != null)
        {
            body.isKinematic = false;
            body.useGravity = true;
        }

        Debug.Log($"[SimGripperPickup] released {attachedObject.name}");
        attachedObject = null;
        attachedOriginalParent = null;
    }
}
