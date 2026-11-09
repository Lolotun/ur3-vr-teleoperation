using System.Linq;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;

public static class WorkshopBuild
{
    public static void BuildAndroidApk()
    {
        BuildApk("build/UR3Workshop_gripperfix.apk");
    }

    public static void BuildAndroidDatasetApk()
    {
        BuildApk("build/UR3Workshop_dataset_buttons.apk");
    }

    private static void BuildApk(string output)
    {
        string[] scenes = EditorBuildSettings.scenes
            .Where(scene => scene.enabled)
            .Select(scene => scene.path)
            .ToArray();

        System.IO.Directory.CreateDirectory("build");
        BuildReport report = BuildPipeline.BuildPlayer(
            scenes,
            output,
            BuildTarget.Android,
            BuildOptions.None
        );
        if (report.summary.result != BuildResult.Succeeded)
            throw new BuildFailedException("APK build failed: " + report.summary.result);
    }
}
