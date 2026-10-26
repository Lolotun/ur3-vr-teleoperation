using UnityEngine;

/// <summary>
/// Spawns a small vertical tower of pickable cubes next to the workshop template cube.
/// </summary>
public class WorkshopCubeTowerSpawner : MonoBehaviour
{
    public Transform templateCube;
    public int towerCubeCount = 3;
    public Vector3 towerBaseOffset = new Vector3(0.12f, 0f, 0f);

    private void Start()
    {
        if (templateCube == null)
        {
            SimPickableObject pickable = FindFirstObjectByType<SimPickableObject>();
            if (pickable != null)
            {
                templateCube = pickable.transform;
            }
        }

        if (templateCube == null)
        {
            Debug.LogWarning("[WorkshopCubeTowerSpawner] No template cube found.", this);
            return;
        }

        float step = GetCubeHeight(templateCube);
        Vector3 worldBase = templateCube.position + templateCube.TransformVector(towerBaseOffset);

        for (int i = 0; i < towerCubeCount; i++)
        {
            GameObject clone = Instantiate(templateCube.gameObject, templateCube.parent);
            clone.name = $"Cube_Tower_{i + 1}";
            Transform cloneTransform = clone.transform;
            cloneTransform.SetPositionAndRotation(worldBase + Vector3.up * step * i, templateCube.rotation);
            cloneTransform.localScale = templateCube.localScale;
        }

        Debug.Log(
            $"[WorkshopCubeTowerSpawner] spawned {towerCubeCount} cubes, step={step:F3} m",
            this
        );
    }

    private static float GetCubeHeight(Transform cube)
    {
        Renderer renderer = cube.GetComponentInChildren<Renderer>();
        return renderer != null ? renderer.bounds.size.y : 0.05f;
    }
}
