using UnityEngine;

public class ForceRobotVisibleMaterial : MonoBehaviour
{
    public Color baseColor = new Color(0.72f, 0.72f, 0.72f, 1f);
    public Color darkColor = new Color(0.08f, 0.08f, 0.08f, 1f);
    public bool includeInactive = true;

    private Material _baseMaterial;
    private Material _darkMaterial;

    private void Awake()
    {
        Shader shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null)
            shader = Shader.Find("Standard");

        _baseMaterial = CreateMaterial(shader, baseColor);
        _darkMaterial = CreateMaterial(shader, darkColor);

        foreach (Renderer renderer in GetComponentsInChildren<Renderer>(includeInactive))
        {
            renderer.enabled = true;
            renderer.sharedMaterial = IsDarkPart(renderer.name) ? _darkMaterial : _baseMaterial;
        }
    }

    private static Material CreateMaterial(Shader shader, Color color)
    {
        var material = new Material(shader)
        {
            name = "Forced UR3 Visible Material",
            color = color,
        };

        if (material.HasProperty("_BaseColor"))
            material.SetColor("_BaseColor", color);
        if (material.HasProperty("_Color"))
            material.SetColor("_Color", color);
        if (material.HasProperty("_Surface"))
            material.SetFloat("_Surface", 0f);
        if (material.HasProperty("_AlphaClip"))
            material.SetFloat("_AlphaClip", 0f);
        if (material.HasProperty("_ZWrite"))
            material.SetFloat("_ZWrite", 1f);

        material.renderQueue = -1;
        return material;
    }

    private static bool IsDarkPart(string objectName)
    {
        string lower = objectName.ToLowerInvariant();
        return lower.Contains("wrist") || lower.Contains("joint") || lower.Contains("collision");
    }
}
