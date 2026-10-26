using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Optional Day 3 feedback: colors TCP_Target when ROS safety clamps workspace.
/// </summary>
[RequireComponent(typeof(Renderer))]
public class SafetyStatusSubscriber : MonoBehaviour
{
    public string topic = "/team_1/safety_status";

    public Color okColor = new Color(0.2f, 0.85f, 0.3f, 1f);
    public Color clampColor = new Color(1f, 0.55f, 0.1f, 1f);
    public Color staleColor = new Color(0.9f, 0.2f, 0.2f, 1f);

    private Renderer targetRenderer;
    private Material instanceMaterial;

    private void Awake()
    {
        targetRenderer = GetComponent<Renderer>();
        instanceMaterial = targetRenderer.material;
    }

    private void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(topic, OnStatus);
        ApplyColor(okColor);
    }

    private void OnStatus(StringMsg msg)
    {
        switch (msg.data)
        {
            case "WORKSPACE_CLAMP":
                ApplyColor(clampColor);
                break;
            case "STALE_HOLD":
            case "IK_ERROR":
            case "IK_NONFINITE":
                ApplyColor(staleColor);
                break;
            default:
                ApplyColor(okColor);
                break;
        }
    }

    private void ApplyColor(Color color)
    {
        if (instanceMaterial != null)
        {
            instanceMaterial.color = color;
        }
    }
}
