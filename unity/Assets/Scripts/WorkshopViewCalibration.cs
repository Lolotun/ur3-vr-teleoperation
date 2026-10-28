using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR;
using RSL.Core.Menu;

// Manual heading alignment, not a measured robot-base or spatial-anchor calibration.
public class WorkshopViewCalibration : MonoBehaviour
{
    public ControllerPosePublisher posePublisher;
    public float distanceToRobot = 1.5f;
    public Vector3 sceneForward = Vector3.forward;

    private XROrigin origin;
    private XRStateManager anchorManager;
    private Button actionButton;
    private TMP_Text actionLabel;
    private TMP_Text hint;
    private GameObject cancelButton;
    private bool calibrating;
    private bool primaryReleased;
    private bool previousManualPlacement;
    private Vector3 savedPosition;
    private Quaternion savedRotation;

    private void Start()
    {
        if (posePublisher == null) posePublisher = GetComponent<ControllerPosePublisher>();
        origin = FindFirstObjectByType<XROrigin>();
        anchorManager = FindFirstObjectByType<XRStateManager>();
        var menu = GameObject.FindGameObjectWithTag("menu");
        var anchor = menu != null ? menu.transform.Find("MenuCanvas/Wifi") as RectTransform : null;
        if (origin == null || posePublisher == null || posePublisher.robotBase == null || anchor == null)
        {
            Debug.LogError("[Calibration] Missing XR origin, robot base or connection panel.", this);
            enabled = false;
            return;
        }

        // Use the existing XR canvas/raycaster, underneath its connection control.
        actionButton = MakeButton(anchor, "Calibration", 0f, out actionLabel);
        actionButton.onClick.AddListener(ToggleCalibration);
        var cancel = MakeButton(anchor, "Cancel", -42f, out _);
        cancel.onClick.AddListener(CancelCalibration);
        cancelButton = cancel.gameObject;
        cancelButton.SetActive(false);
        hint = MakeText(actionButton.transform, "", 140f, 64f);
        var hintRect = hint.rectTransform;
        hintRect.anchoredPosition = new Vector2(0f, -112f);
        hint.fontSize = 14f;
    }

    private Button MakeButton(RectTransform anchor, string label, float offset, out TMP_Text text)
    {
        var go = new GameObject(label, typeof(RectTransform), typeof(Image), typeof(Button));
        go.layer = anchor.gameObject.layer;
        var rect = (RectTransform)go.transform;
        rect.SetParent(anchor.parent, false);
        rect.anchorMin = anchor.anchorMin;
        rect.anchorMax = anchor.anchorMax;
        rect.pivot = anchor.pivot;
        rect.localRotation = anchor.localRotation;
        rect.localScale = anchor.localScale;
        rect.sizeDelta = new Vector2(140f, 36f);
        rect.localPosition = anchor.localPosition + anchor.localRotation *
            Vector3.Scale(anchor.localScale, new Vector3(0f, -100f + offset, 0f));
        go.GetComponent<Image>().color = new Color(0.08f, 0.22f, 0.32f, 1f);
        text = MakeText(rect, label, 140f, 36f);
        var template = anchor.GetComponentInChildren<TMP_Text>(true);
        if (template != null) text.font = template.font;
        return go.GetComponent<Button>();
    }

    private static TMP_Text MakeText(Transform parent, string value, float width, float height)
    {
        var go = new GameObject("Label", typeof(RectTransform), typeof(TextMeshProUGUI));
        go.layer = parent.gameObject.layer;
        go.transform.SetParent(parent, false);
        var label = go.GetComponent<TextMeshProUGUI>();
        label.text = value;
        label.fontSize = 18f;
        label.color = Color.white;
        label.alignment = TextAlignmentOptions.Center;
        label.raycastTarget = false;
        label.rectTransform.sizeDelta = new Vector2(width, height);
        return label;
    }

    public void ToggleCalibration()
    {
        if (calibrating)
        {
            ApplyCalibration();
            return;
        }
        savedPosition = origin.transform.position;
        savedRotation = origin.transform.rotation;
        previousManualPlacement = anchorManager != null && anchorManager.ManualPlacement;
        if (anchorManager != null) anchorManager.ManualPlacement = true;
        posePublisher.SetCalibrationBlocked(true);
        calibrating = true;
        primaryReleased = false;
        actionLabel.text = "Apply";
        cancelButton.SetActive(true);
        hint.text = "Face the real robot.\nPress A or Apply.\nRobot input paused.";
    }

    private void Update()
    {
        if (!calibrating) return;
        var hand = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        if (!hand.isValid || !hand.TryGetFeatureValue(CommonUsages.primaryButton, out bool pressed)) return;
        if (!pressed) primaryReleased = true;
        if (pressed && primaryReleased) ApplyCalibration();
    }

    private void ApplyCalibration()
    {
        var camera = origin.Camera;
        if (camera == null || !camera.isActiveAndEnabled) return;
        Vector3 gaze = Vector3.ProjectOnPlane(camera.transform.forward, Vector3.up);
        Vector3 forward = Vector3.ProjectOnPlane(sceneForward, Vector3.up);
        if (gaze.sqrMagnitude < 0.01f || forward.sqrMagnitude < 0.01f) return;
        forward.Normalize();
        Quaternion correction = Quaternion.FromToRotation(gaze.normalized, forward);
        Vector3 head = camera.transform.position;
        Vector3 desiredHead = posePublisher.robotBase.position - forward * Mathf.Max(0.5f, distanceToRobot);
        desiredHead.y = head.y; // Preserve floor/eye height; yaw only, no scale changes.
        origin.transform.SetPositionAndRotation(
            desiredHead + correction * (origin.transform.position - head),
            correction * origin.transform.rotation);
        Finish("Aligned for this session.\nRelease Trigger before control.");
        Debug.Log("[Calibration] View aligned. Robot base axes still require preview verification.", this);
    }

    public void CancelCalibration()
    {
        if (!calibrating) return;
        origin.transform.SetPositionAndRotation(savedPosition, savedRotation);
        if (anchorManager != null) anchorManager.ManualPlacement = previousManualPlacement;
        Finish("Calibration cancelled.");
    }

    private void Finish(string message)
    {
        calibrating = false;
        posePublisher.SetCalibrationBlocked(false);
        actionLabel.text = "Calibration";
        cancelButton.SetActive(false);
        hint.text = message;
    }

    private void OnDisable()
    {
        if (calibrating) CancelCalibration();
    }
}
