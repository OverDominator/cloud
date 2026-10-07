using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

// Randomize once per scene load; never move hazards during a running trial.
[DefaultExecutionOrder(-500)]
public class SimpleSceneLayout : MonoBehaviour
{
    public bool randomizeEachRun = true;
    public bool easyExperiment = false;
    public int seed = 27001;
    [Range(0f, 3f)] public float offsetRadius = 2f;
    [Min(0f)] public float clearance = 2.5f;
    [Serializable] public class Placement { public string name; public Vector3 position; public Vector3 scale; }
    [Serializable] public class Layout
    {
        public string version = "simple_v5_acoustic_relay";
        public int seed;
        public float radius;
        public List<Placement> objects = new List<Placement>();
    }
    string snapshot;
    ROSConnection ros;
    float nextPublish;

    void Start()
    {
        string sceneName = SceneManager.GetActiveScene().name;
        if (sceneName != "FireRescue_Simplified" && sceneName != "FireRescue_Easy") return;
        var obstacle = GameObject.Find("Obstacle_A");
        var obstacleB = GameObject.Find("Obstacle_B");
        var fire = GameObject.Find("FireSource_01");
        if (obstacle == null || obstacleB == null || fire == null)
        { Debug.LogError("Random layout requires Obstacle_A, Obstacle_B and FireSource_01."); return; }
        int usedSeed = randomizeEachRun ? Guid.NewGuid().GetHashCode() : seed;
        var rng = new System.Random(usedSeed);
        var layout = new Layout { seed = usedSeed, radius = offsetRadius };
        if (easyExperiment) layout.version = "easy_v1_acoustic_relay";
        // Create the relay before VictimCallSensor.Start freezes the total.
        var victim1 = GameObject.Find("Victim_01");
        var victim2 = GameObject.Find("Victim_02");
        if (victim1 == null || victim2 == null)
        { Debug.LogError("Relay layout requires both authored victims."); return; }
        var victims = new List<GameObject> { victim1, victim2 };
        for (int i = 3; i <= 4; i++)
        {
            var copy = Instantiate(victim1, victim1.transform.parent);
            copy.name = "Victim_0" + i;
            victims.Add(copy);
        }
        Vector2[] relay = { new Vector2(-28,-2), new Vector2(-22,18),
            new Vector2(5,4), new Vector2(32,0) };
        if (easyExperiment)
            relay = new[] { new Vector2(-28,-2), new Vector2(-22,16),
                new Vector2(0,2), new Vector2(22,0) };
        for (int i = 0; i < victims.Count; i++)
        {
            var victim = victims[i];
            victim.transform.position = new Vector3(relay[i].x, victim.transform.position.y, relay[i].y);
            layout.objects.Add(new Placement { name = victim.name, position = victim.transform.position, scale = victim.transform.lossyScale });
        }
        var sensor = FindFirstObjectByType<VictimCallSensor>();
        if (sensor != null) sensor.acousticSeed = usedSeed;
        var objects = new List<GameObject> { obstacle, obstacleB };
        for (int i = 0; i < 3; i++)
        {
            var copy = Instantiate(obstacle, transform);
            copy.name = "Obstacle_" + (char)('C' + i);
            objects.Add(copy);
        }
        objects.Add(fire);
        var secondFire = Instantiate(fire, fire.transform.parent);
        secondFire.name = "FireSource_02";
        objects.Add(secondFire);
        Vector2[] centers = { new Vector2(8,-23), new Vector2(28,-7),
            new Vector2(-23,-8), new Vector2(3,22), new Vector2(31,19),
            new Vector2(15,-15), new Vector2(-23,27) };
        // Inactive pending objects must not interfere with rejection sampling.
        foreach (var item in objects) item.SetActive(false);
        for (int i = 0; i < objects.Count; i++)
        {
            var item = objects[i];
            item.SetActive(true);
            Physics.SyncTransforms();
            Bounds extent = new Bounds(item.transform.position, Vector3.zero);
            foreach (var col in item.GetComponentsInChildren<Collider>()) extent.Encapsulate(col.bounds);
            float radius = Mathf.Max(extent.extents.x, extent.extents.z);
            bool placed = false;
            for (int attempt = 0; attempt < 120; attempt++)
            {
                double angle = rng.NextDouble() * Math.PI * 2;
                float distance = offsetRadius * Mathf.Sqrt((float)rng.NextDouble());
                var p = new Vector3(centers[i].x + distance * (float)Math.Cos(angle),
                    item.transform.position.y, centers[i].y + distance * (float)Math.Sin(angle));
                if (!Clear(item, p, radius)) continue;
                item.transform.position = p;
                Physics.SyncTransforms();
                placed = true;
                layout.objects.Add(new Placement { name = item.name, position = p, scale = item.transform.lossyScale });
                break;
            }
            if (!placed)
            {
                Debug.LogError("Layout placement failed for " + item.name + "; trial disabled. Reduce radius/clearance or change centers.");
                foreach (var placedObject in objects) placedObject.SetActive(false);
                enabled = false;
                // Do not silently run an experiment with missing hazards.
                Time.timeScale = 0;
                return;
            }
        }
        snapshot = JsonUtility.ToJson(layout, true);
        string dir = Path.Combine(Application.persistentDataPath, "SceneLayouts");
        Directory.CreateDirectory(dir);
        File.WriteAllText(Path.Combine(dir, DateTime.Now.ToString("yyyyMMdd_HHmmss_fff") + "_" + usedSeed + ".json"), snapshot);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>("/experiment/layout", 1);
        Debug.Log("Relay map: 4 victims, 5 obstacles, 2 fires; layout/acoustic seed=" + usedSeed);
    }

    bool Clear(GameObject item, Vector3 p, float radius)
    {
        if (Mathf.Abs(p.x) + radius + clearance > 44.5f || Mathf.Abs(p.z) + radius + clearance > 34.5f) return false;
        foreach (var col in FindObjectsByType<Collider>(FindObjectsSortMode.None))
        {
            if (!col.enabled || col.isTrigger || col.transform.IsChildOf(item.transform)) continue;
            Bounds b = col.bounds;
            if (b.max.y <= 0.1f) continue; // Floor is not a lateral obstruction.
            float dx = Mathf.Max(b.min.x - p.x, 0, p.x - b.max.x);
            float dz = Mathf.Max(b.min.z - p.z, 0, p.z - b.max.z);
            if (dx * dx + dz * dz < (radius + clearance) * (radius + clearance)) return false;
        }
        foreach (string actor in new[] { "TrackedRobot", "Victim_01", "Victim_02", "Victim_03", "Victim_04" })
        {
            var obj = GameObject.Find(actor);
            if (obj != null && Vector2.Distance(new Vector2(p.x,p.z), new Vector2(obj.transform.position.x,obj.transform.position.z)) < radius + 5f) return false;
        }
        return true;
    }

    void Update()
    {
        if (ros == null || snapshot == null || Time.unscaledTime < nextPublish) return;
        nextPublish = Time.unscaledTime + 2f;
        ros.Publish("/experiment/layout", new StringMsg(snapshot));
    }
}
