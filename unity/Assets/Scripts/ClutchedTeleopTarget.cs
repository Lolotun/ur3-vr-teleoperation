using UnityEngine;
using UnityEngine.XR;

public class ClutchedTeleopTarget : MonoBehaviour
{
    public Transform rightController;
    public Transform initialPoseSource;
    public bool snapToInitialPoseSourceOnStart;
    public bool snapToInitialPoseSourceOnClutchBegin;
    public Transform workspaceFrame;
    public Vector3 minLocal = new Vector3(-0.30f, 0.00f, -0.25f);
    public Vector3 maxLocal = new Vector3(0.30f, 0.35f, 0.25f);
    public float maxLinearSpeed = 0.40f;
    public float maxAngularSpeedDeg = 180f;

    private InputDevice rightHandDevice;
    private bool wasTriggerPressed;
    private bool clutchActive;

    private Vector3 clutchStartControllerPosition;
    private Quaternion clutchStartControllerRotation;
    private Vector3 clutchStartTargetPosition;
    private Quaternion clutchStartTargetRotation;

    public bool IsClutchActive => clutchActive;

    private void Start()
    {
        if (snapToInitialPoseSourceOnStart && initialPoseSource != null)
        {
            transform.position = initialPoseSource.position;
            transform.rotation = initialPoseSource.rotation;
        }
    }

    private void Update()
    {
        bool triggerPressed = IsRightTriggerPressed();

        if (triggerPressed && !wasTriggerPressed)
        {
            BeginClutch();
        }

        if (triggerPressed && clutchActive)
        {
            UpdateTargetPose();
        }

        if (!triggerPressed)
        {
            clutchActive = false;
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

    private void BeginClutch()
    {
        Transform controller = GetControllerTransform();
        if (controller == null)
        {
            return;
        }

        clutchStartControllerPosition = controller.position;
        clutchStartControllerRotation = controller.rotation;
        clutchStartTargetPosition = transform.position;
        clutchStartTargetRotation = transform.rotation;
        clutchActive = true;
    }

    private void UpdateTargetPose()
    {
        Transform controller = GetControllerTransform();
        if (controller == null)
        {
            return;
        }

        Vector3 controllerDelta = controller.position - clutchStartControllerPosition;
        Quaternion controllerRotationDelta = controller.rotation * Quaternion.Inverse(clutchStartControllerRotation);

        Vector3 desiredPosition = clutchStartTargetPosition + controllerDelta;
        Quaternion desiredRotation = controllerRotationDelta * clutchStartTargetRotation;

        desiredPosition = ClampToWorkspace(desiredPosition);

        float maxDistanceDelta = Mathf.Max(0f, maxLinearSpeed) * Time.deltaTime;
        float maxDegreesDelta = Mathf.Max(0f, maxAngularSpeedDeg) * Time.deltaTime;

        transform.position = Vector3.MoveTowards(transform.position, desiredPosition, maxDistanceDelta);
        transform.rotation = Quaternion.RotateTowards(transform.rotation, desiredRotation, maxDegreesDelta);
    }

    private Transform GetControllerTransform()
    {
        return rightController;
    }

    private Vector3 ClampToWorkspace(Vector3 targetWorldPosition)
    {
        if (workspaceFrame == null)
        {
            return targetWorldPosition;
        }

        Vector3 localPosition = workspaceFrame.InverseTransformPoint(targetWorldPosition);
        localPosition.x = Mathf.Clamp(localPosition.x, minLocal.x, maxLocal.x);
        localPosition.y = Mathf.Clamp(localPosition.y, minLocal.y, maxLocal.y);
        localPosition.z = Mathf.Clamp(localPosition.z, minLocal.z, maxLocal.z);

        return workspaceFrame.TransformPoint(localPosition);
    }

    private void OnDrawGizmosSelected()
    {
        if (workspaceFrame == null)
        {
            return;
        }

        Vector3 center = (minLocal + maxLocal) * 0.5f;
        Vector3 size = maxLocal - minLocal;

        Matrix4x4 previousMatrix = Gizmos.matrix;
        Color previousColor = Gizmos.color;

        Gizmos.matrix = workspaceFrame.localToWorldMatrix;
        Gizmos.color = new Color(1f, 0.92f, 0.05f, 0.35f);
        Gizmos.DrawWireCube(center, size);

        Gizmos.matrix = previousMatrix;
        Gizmos.color = previousColor;
    }
}
