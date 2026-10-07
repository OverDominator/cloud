using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class EasyFireSceneBuilder
{
    [MenuItem("Tools/Fire Rescue/Create Easy Rescue Experiment Scene")]
    public static void CreateScene()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            EditorUtility.DisplayDialog("Fire Rescue", "Stop Play Mode first.", "OK");
            return;
        }
        if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
        const string output = "Assets/Scenes/FireRescue_Easy.unity";
        if (System.IO.File.Exists(output) && !EditorUtility.DisplayDialog(
            "Fire Rescue", "Replace the existing Easy scene?", "Replace", "Cancel")) return;
        var scene = EditorSceneManager.OpenScene("Assets/Scenes/FireRescue_Simplified.unity");
        // Save under a new path before modifying any environment objects.
        if (!EditorSceneManager.SaveScene(scene, output))
            throw new System.IO.IOException("Cannot create " + output);
        var host = GameObject.Find("SimplifiedFireScene");
        var layout = host.GetComponent<SimpleSceneLayout>();
        layout.easyExperiment = true;
        layout.offsetRadius = 0.5f;
        foreach (string name in new[] { "Divider_East", "Upper_Room_East" })
        {
            var wall = host.transform.Find(name);
            if (wall != null) Object.DestroyImmediate(wall.gameObject);
        }
        var west = host.transform.Find("Divider_West");
        if (west != null)
        {
            west.position = new Vector3(-8f, west.position.y, -29f);
            west.localScale = new Vector3(1f, west.localScale.y, 12f);
        }
        var south = host.transform.Find("Upper_Room_South");
        if (south != null)
        {
            south.position = new Vector3(14f, south.position.y, 14f);
            south.localScale = new Vector3(10f, south.localScale.y, 1f);
        }
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
            throw new System.IO.IOException("Cannot save " + output);
        if (!EditorBuildSettings.scenes.Any(s => s.path == output))
            EditorBuildSettings.scenes = EditorBuildSettings.scenes.Concat(
                new[] { new EditorBuildSettingsScene(output, true) }).ToArray();
        Selection.activeGameObject = host;
        Debug.Log("Easy experiment created: 4 victims, 5 obstacles, 2 fires; jitter 0.5m; easy_v1_acoustic_relay.");
    }
}
