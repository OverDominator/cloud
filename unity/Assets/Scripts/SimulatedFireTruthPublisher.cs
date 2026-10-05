using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

// The thesis simple scene has authored, movable fire sources. Publish their
// map positions as simulation ground truth for the navigation keepout layer.
// Image detection remains a separate perception/confirmation experiment.
[DefaultExecutionOrder(-450)]
public class SimulatedFireTruthPublisher : MonoBehaviour
{
    const string Topic = "/fire_detection/simulated_truth";
    ROSConnection ros;
    float nextPublish;

    void Start()
    {
        if (SceneManager.GetActiveScene().name != "FireRescue_Simplified" &&
            SceneManager.GetActiveScene().name != "FireRescue_Easy")
        {
            enabled = false;
            return;
        }
        ros = ROSConnection.GetOrCreateInstance();
        // All fire sources are published in one frame; retain the whole burst.
        ros.RegisterPublisher<PointStampedMsg>(Topic, 16);
    }

    void Update()
    {
        if (ros == null || !ros.HasConnectionThread || ros.HasConnectionError ||
            Time.unscaledTime < nextPublish) return;
        nextPublish = Time.unscaledTime + 0.5f;
        var seen = new HashSet<int>();
        foreach (var fire in GameObject.FindGameObjectsWithTag("FireSource"))
        {
            Transform root = fire.transform;
            while (root.parent != null && root.parent.CompareTag("FireSource")) root = root.parent;
            if (!seen.Add(root.gameObject.GetInstanceID()) || !root.name.StartsWith("FireSource")) continue;
            long ms = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            ros.Publish(Topic, new PointStampedMsg {
                header = new HeaderMsg {
                    frame_id = "map",
                    stamp = new TimeMsg((uint)(ms / 1000), (uint)((ms % 1000) * 1000000))
                },
                point = root.position.To<FLU>()
            });
        }
    }
}

public static class SimulatedFireTruthPublisherBootstrap
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Attach()
    {
        if (SceneManager.GetActiveScene().name != "FireRescue_Simplified" &&
            SceneManager.GetActiveScene().name != "FireRescue_Easy") return;
        var host = GameObject.Find("SimplifiedFireScene");
        if (host != null && host.GetComponent<SimulatedFireTruthPublisher>() == null)
            host.AddComponent<SimulatedFireTruthPublisher>();
    }
}
