using UnityEngine;

/// <summary>
/// Victims and fire sources remain visible to LiDAR raycasts, but they do not
/// physically pin the mobile robot. Navigation avoidance is handled by ROS.
/// </summary>
public static class RescueCollisionPolicy
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void ConfigureLayerCollisions()
    {
        int robotLayer = LayerMask.NameToLayer("Robot");
        int victimLayer = LayerMask.NameToLayer("Victim");
        int fireLayer = LayerMask.NameToLayer("FireSource");

        if (robotLayer < 0 || victimLayer < 0 || fireLayer < 0)
        {
            Debug.LogError(
                "RescueCollisionPolicy requires Robot, Victim, and FireSource layers.");
            return;
        }

        Physics.IgnoreLayerCollision(robotLayer, victimLayer, true);
        Physics.IgnoreLayerCollision(robotLayer, fireLayer, true);
        Debug.Log(
            "Robot collisions with Victim and FireSource disabled; LiDAR raycasts remain enabled.");
    }
}
