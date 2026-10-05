using UnityEngine;

public static class RunInBackground
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void Enable()
    {
        Application.runInBackground = true;
        QualitySettings.vSyncCount = 0;
        QualitySettings.antiAliasing = 0;
        QualitySettings.shadows = ShadowQuality.Disable;
        QualitySettings.shadowDistance = 0f;
        Application.targetFrameRate = 24;
        Debug.Log("[Runtime] Low-load background mode enabled; 24 FPS, shadows disabled.");
    }
}
