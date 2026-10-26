using System.Linq;
using UnityEditor;

public static class WorkshopBuild
{
    public static void BuildAndroidApk()
    {
        string[] scenes = EditorBuildSettings.scenes
            .Where(scene => scene.enabled)
            .Select(scene => scene.path)
            .ToArray();

        BuildPipeline.BuildPlayer(
            scenes,
            "build/UR3Workshop_gripperfix.apk",
            BuildTarget.Android,
            BuildOptions.None
        );
    }
}
