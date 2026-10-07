using System.IO;
using UnityEditor;

[InitializeOnLoad]
public static class FireBatchPlayBridge
{
    static double nextTick;
    static FireBatchPlayBridge() { EditorApplication.update += Tick; }
    static void Tick()
    {
        if (EditorApplication.timeSinceStartup < nextTick) return;
        nextTick = EditorApplication.timeSinceStartup + 0.5;
        if (EditorApplication.isCompiling || EditorApplication.isUpdating) return;
        const string commandPath = "Library/FireBatchPlay.command";
        if (File.Exists(commandPath))
        {
            string command = File.ReadAllText(commandPath).Trim();
            File.Delete(commandPath);
            if (command == "stop") EditorApplication.isPlaying = false;
            if (command == "expandhouse" && !EditorApplication.isPlayingOrWillChangePlaymode)
                ExpandedHouseFurniture.Build();
            if (command == "house" && !EditorApplication.isPlayingOrWillChangePlaymode)
                SingleStoreyHouseBuilder.Build();
            if (command.StartsWith("pilot:") && !EditorApplication.isPlayingOrWillChangePlaymode)
                MultiroomExperiment.RunFor(int.Parse(command.Substring(6)));
            if (command == "furnish" && !EditorApplication.isPlayingOrWillChangePlaymode)
                FurnishedRescueScene.Build();
            if (command == "play" && !EditorApplication.isPlayingOrWillChangePlaymode)
                EditorApplication.isPlaying = true;
        }
        File.WriteAllText("Library/FireBatchPlayState.txt",
            EditorApplication.isPlaying ? "playing" : "stopped");
    }
}
