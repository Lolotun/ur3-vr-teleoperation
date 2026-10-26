using UnityEngine;

public class FramePoseDebug : MonoBehaviour
{
    public string frameName = "base_link";
    public bool logOnStart = true;
    public KeyCode logKey = KeyCode.F9;

    private void Start()
    {
        if (logOnStart)
        {
            LogPose();
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(logKey))
        {
            LogPose();
        }
    }

    [ContextMenu("Log Frame Pose")]
    public void LogPose()
    {
        Vector3 unityPosition = transform.position;
        Vector3 rosPosition = UnityToRosFlu(unityPosition);
        Vector3 rosRight = UnityToRosFlu(transform.right);
        Vector3 rosUp = UnityToRosFlu(transform.up);
        Vector3 rosForward = UnityToRosFlu(transform.forward);

        Debug.LogWarning(
            $"[{frameName}] Unity pos={Format(unityPosition)} euler={Format(transform.eulerAngles)}\n" +
            $"[{frameName}] ROS FLU pos={Format(rosPosition)}\n" +
            $"[{frameName}] Unity axes in ROS FLU: right={Format(rosRight)} up={Format(rosUp)} forward={Format(rosForward)}"
        );
    }

    private static string Format(Vector3 value)
    {
        return $"({value.x:F4}, {value.y:F4}, {value.z:F4})";
    }

    private static Vector3 UnityToRosFlu(Vector3 value)
    {
        return new Vector3(value.z, -value.x, value.y);
    }
}
