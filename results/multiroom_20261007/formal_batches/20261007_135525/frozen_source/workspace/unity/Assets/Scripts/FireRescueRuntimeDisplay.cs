using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

[DisallowMultipleComponent]
public class FireRescueRuntimeDisplay : MonoBehaviour
{
    [Header("Rescue HUD")]
    public string rescuedCountTopic = "/victim_mission/rescued_count";
    public string missionStateTopic = "/victim_mission/state";
    public string missionStatusTopic = "/victim_mission/status";
    public string recoveryStateTopic = "/local_recovery/state";
    public string totalVictimsTopic = "/victim_call/total";
    public int totalVictims = 5;

    [Header("Robot trajectory")]
    public float trajectorySampleDistance = 0.50f;
    public int maximumTrajectoryPoints = 3000;
    public float trajectoryHeight = 0.12f;
    public float trajectoryWidth = 0.16f;
    public Color trajectoryColor = new Color(0f, 0.95f, 1f, 0.95f);

    [Header("Detected structure labels")]
    public float labelHeight = 0.8f;
    public float labelCharacterSize = 0.32f;
    public int maximumLabels = 80;
    public Color wallLabelColor = new Color(0.2f, 0.85f, 1f, 1f);
    public Color obstacleLabelColor = new Color(1f, 0.65f, 0.1f, 1f);

    private readonly List<Vector3> trajectory = new List<Vector3>();
    private readonly HashSet<Collider> labelledColliders = new HashSet<Collider>();
    private readonly List<Transform> labelTransforms = new List<Transform>();
    private ROSConnection ros;
    private LineRenderer trajectoryLine;
    private LidarSensor lidar;
    private CmdVelSubscriber motionController;
    private Camera displayCamera;
    private int rescuedCount;
    private string missionState = "STARTING";
    private string missionStatus = "waiting for ROS";
    private string recoveryState = "normal";
    private GUIStyle hudStyle;
    private GUIStyle hudShadowStyle;
    private bool completionPopup;
    private bool savedThisRun;
    private bool resetPending;
    private bool reloadScenePending;
    private string saveReceiptPath;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Int32Msg>(rescuedCountTopic, OnRescuedCount);
        ros.Subscribe<StringMsg>(missionStateTopic, OnMissionState);
        ros.Subscribe<StringMsg>(missionStatusTopic, OnMissionStatus);
        ros.Subscribe<StringMsg>(recoveryStateTopic, OnRecoveryState);
        ros.Subscribe<Int32Msg>(totalVictimsTopic, OnTotalVictims);
        ros.RegisterRosService<
            RosMessageTypes.Std.EmptyRequest,
            RosMessageTypes.Std.EmptyResponse>("/victim_mission/reset");

        totalVictims = CountVictimRoots();

        lidar = FindFirstObjectByType<LidarSensor>();
        motionController = GetComponent<CmdVelSubscriber>();
        displayCamera = Camera.main;
        ConfigureSensorExclusion();
        ConfigureTrajectory();
        AddTrajectoryPoint(transform.position);
    }

    void Update()
    {
        if (reloadScenePending)
        {
            reloadScenePending = false;
            Scene activeScene = SceneManager.GetActiveScene();
            if (activeScene.buildIndex >= 0)
                SceneManager.LoadScene(activeScene.buildIndex);
            else
                SceneManager.LoadScene(activeScene.name);
            return;
        }
        SampleTrajectory();
        UpdateDetectedStructureLabels();
        if (!completionPopup && !savedThisRun &&
            (missionState.Equals("COMPLETE", StringComparison.OrdinalIgnoreCase) ||
             missionStatus.StartsWith("mission complete", StringComparison.OrdinalIgnoreCase)))
            completionPopup = true;
    }

    void LateUpdate()
    {
        if (displayCamera == null) displayCamera = Camera.main;
        if (displayCamera == null) return;

        // Keep the text horizontal and readable from the operator camera.
        Quaternion facing = displayCamera.transform.rotation;
        foreach (Transform labelTransform in labelTransforms)
        {
            if (labelTransform != null) labelTransform.rotation = facing;
        }
    }

    void OnRescuedCount(Int32Msg message)
    {
        rescuedCount = Mathf.Clamp(message.data, 0, Mathf.Max(0, totalVictims));
    }

    void OnTotalVictims(Int32Msg message)
    {
        if (message.data <= 0) return;
        totalVictims = message.data;
        rescuedCount = Mathf.Clamp(rescuedCount, 0, totalVictims);
    }

    static int CountVictimRoots()
    {
        try
        {
            HashSet<int> roots = new HashSet<int>();
            foreach (GameObject victim in GameObject.FindGameObjectsWithTag("Victim"))
            {
                Transform root = victim.transform;
                while (root.parent != null && root.parent.CompareTag("Victim"))
                    root = root.parent;
                roots.Add(root.gameObject.GetInstanceID());
            }
            return roots.Count;
        }
        catch (UnityException)
        {
            return 0;
        }
    }

    void OnMissionState(StringMsg message)
    {
        missionState = string.IsNullOrWhiteSpace(message.data)
            ? "UNKNOWN" : message.data;
    }

    void OnMissionStatus(StringMsg message)
    {
        missionStatus = string.IsNullOrWhiteSpace(message.data)
            ? "-" : message.data;
    }

    void OnRecoveryState(StringMsg message)
    {
        recoveryState = string.IsNullOrWhiteSpace(message.data)
            ? "unknown" : message.data;
    }

    void ConfigureTrajectory()
    {
        trajectoryLine = gameObject.GetComponent<LineRenderer>();
        if (trajectoryLine == null) trajectoryLine = gameObject.AddComponent<LineRenderer>();

        trajectoryLine.useWorldSpace = true;
        trajectoryLine.loop = false;
        trajectoryLine.startWidth = trajectoryWidth;
        trajectoryLine.endWidth = trajectoryWidth;
        trajectoryLine.startColor = trajectoryColor;
        trajectoryLine.endColor = trajectoryColor;
        trajectoryLine.numCornerVertices = 4;
        trajectoryLine.numCapVertices = 4;
        trajectoryLine.positionCount = 0;
        trajectoryLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        trajectoryLine.receiveShadows = false;

        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null) shader = Shader.Find("Unlit/Color");
        if (shader != null)
        {
            Material material = new Material(shader);
            material.color = trajectoryColor;
            trajectoryLine.material = material;
        }
    }

    void SampleTrajectory()
    {
        Vector3 point = transform.position;
        if (trajectory.Count == 0 ||
            Vector3.Distance(trajectory[trajectory.Count - 1], point) >= trajectorySampleDistance)
        {
            AddTrajectoryPoint(point);
        }
    }

    void AddTrajectoryPoint(Vector3 position)
    {
        Vector3 visiblePosition = position + Vector3.up * trajectoryHeight;
        trajectory.Add(visiblePosition);

        if (trajectory.Count > maximumTrajectoryPoints)
        {
            trajectory.RemoveAt(0);
            trajectoryLine.positionCount = trajectory.Count;
            trajectoryLine.SetPositions(trajectory.ToArray());
            return;
        }

        trajectoryLine.positionCount = trajectory.Count;
        trajectoryLine.SetPosition(trajectory.Count - 1, visiblePosition);
    }

    void UpdateDetectedStructureLabels()
    {
        if (lidar == null)
        {
            lidar = FindFirstObjectByType<LidarSensor>();
            if (lidar == null) return;
        }

        foreach (Collider detectedCollider in lidar.DetectedEnvironmentColliders)
        {
            if (labelledColliders.Count >= maximumLabels) return;
            if (detectedCollider == null || labelledColliders.Contains(detectedCollider)) continue;

            labelledColliders.Add(detectedCollider);
            if (ShouldIgnoreStructure(detectedCollider)) continue;
            CreateStructureLabel(detectedCollider);
        }
    }

    bool ShouldIgnoreStructure(Collider detectedCollider)
    {
        string objectName = detectedCollider.gameObject.name.ToLowerInvariant();
        if (objectName.Contains("floor") || objectName.Contains("ground") ||
            objectName.Contains("ceiling") || objectName.Contains("roof"))
        {
            return true;
        }

        Bounds bounds = detectedCollider.bounds;
        return bounds.size.y < 0.35f && bounds.size.x > 3f && bounds.size.z > 3f;
    }

    void CreateStructureLabel(Collider detectedCollider)
    {
        bool isWall = IsWallLike(detectedCollider);
        GameObject labelObject = new GameObject(
            "Detected_" + (isWall ? "Wall_" : "Obstacle_") + detectedCollider.gameObject.name);

        // Ignore Raycast keeps the marker out of LiDAR. Thermal camera also
        // excludes this layer below, so visualization cannot become sensor data.
        labelObject.layer = LayerMask.NameToLayer("Ignore Raycast");
        Bounds bounds = detectedCollider.bounds;
        labelObject.transform.position = new Vector3(
            bounds.center.x,
            bounds.max.y + labelHeight,
            bounds.center.z);

        TextMesh text = labelObject.AddComponent<TextMesh>();
        text.text = isWall ? "WALL" : "OBSTACLE";
        text.anchor = TextAnchor.MiddleCenter;
        text.alignment = TextAlignment.Center;
        text.characterSize = labelCharacterSize;
        text.fontSize = 48;
        text.color = isWall ? wallLabelColor : obstacleLabelColor;

        MeshRenderer renderer = labelObject.GetComponent<MeshRenderer>();
        if (renderer != null)
        {
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            renderer.receiveShadows = false;
        }
        labelTransforms.Add(labelObject.transform);
    }

    static bool IsWallLike(Collider collider)
    {
        if (collider == null) return false;
        Bounds bounds = collider.bounds;
        string[] wallWords =
        {
            "wall", "boundary", "divider", "barrier", "wing", "room"
        };
        // Colliders are often on a generated child named Cube/Box. Inspect
        // the complete hierarchy so a WallEnd child is not mislabeled as a
        // generic obstacle (and never infer a cylinder from a round-looking
        // point-cloud projection).
        for (Transform node = collider.transform; node != null; node = node.parent)
        {
            string lowerName = node.name.ToLowerInvariant();
            foreach (string word in wallWords)
                if (lowerName.Contains(word)) return true;
        }
        return bounds.size.y > 1.5f && Mathf.Max(bounds.size.x, bounds.size.z) >
            2.5f * Mathf.Min(bounds.size.x, bounds.size.z);
    }

    void ConfigureSensorExclusion()
    {
        int ignoreRaycastLayer = LayerMask.NameToLayer("Ignore Raycast");
        if (ignoreRaycastLayer < 0) return;
        int keepMask = ~(1 << ignoreRaycastLayer);

        GameObject[] thermalObjects;
        try
        {
            thermalObjects = GameObject.FindGameObjectsWithTag("ThermalCamera");
        }
        catch (UnityException)
        {
            return;
        }

        foreach (GameObject thermalObject in thermalObjects)
        {
            Camera thermalCamera = thermalObject.GetComponent<Camera>();
            if (thermalCamera != null) thermalCamera.cullingMask &= keepMask;
        }
    }

    void OnGUI()
    {
        EnsureHudStyles();
        string shortStatus = missionStatus.Length > 54
            ? missionStatus.Substring(0, 51) + "..." : missionStatus;
        string text =
            $"救援人数  {rescuedCount} / {totalVictims}\n" +
            $"任务状态  {missionState}\n" +
            $"恢复状态  {recoveryState}\n" +
            $"说明  {shortStatus}";
        text += "\n执行状态  " + (motionController != null ? motionController.MotionStatus : "unavailable");
        Rect panel = new Rect(22f, 22f, 660f, 160f);
        GUI.Box(panel, GUIContent.none);
        GUI.Label(new Rect(panel.x + 20f, panel.y + 10f, 625f, 144f), text, hudShadowStyle);
        GUI.Label(new Rect(panel.x + 18f, panel.y + 8f, 625f, 144f), text, hudStyle);

        if (!completionPopup) return;
        Rect popup = new Rect(Screen.width * 0.5f - 250f, Screen.height * 0.5f - 105f, 500f, 210f);
        GUI.Box(popup, GUIContent.none);
        GUI.Label(new Rect(popup.x + 24f, popup.y + 18f, 450f, 34f),
            "本轮任务已完成", hudStyle);
        GUI.Label(new Rect(popup.x + 24f, popup.y + 58f, 450f, 28f),
            savedThisRun ? "本轮数据已确认保存" : "请选择保存本轮数据，或重置后重新测试", hudStyle);

        GUI.enabled = !savedThisRun && !resetPending;
        if (GUI.Button(new Rect(popup.x + 24f, popup.y + 112f, 205f, 55f), "保存本轮数据"))
            ConfirmSave();
        GUI.enabled = !resetPending;
        if (GUI.Button(new Rect(popup.x + 270f, popup.y + 112f, 205f, 55f),
            resetPending ? "正在重置..." : "重置本轮"))
            RequestReset();
        GUI.enabled = true;
    }

    void ConfirmSave()
    {
        // The ROS logger seals COMPLETE runs automatically. This receipt makes
        // the operator's explicit save decision auditable without duplicating
        // or rewriting the already sealed trajectory bundle.
        string directory = System.IO.Path.Combine(Application.persistentDataPath, "FireRescueSaveReceipts");
        System.IO.Directory.CreateDirectory(directory);
        saveReceiptPath = System.IO.Path.Combine(directory,
            "complete_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".txt");
        System.IO.File.WriteAllText(saveReceiptPath,
            "mission=complete\nrescued=" + rescuedCount + "\n" +
            "total=" + totalVictims + "\nstatus=" + missionStatus + "\n");
        savedThisRun = true;
        completionPopup = true;
        Debug.Log("Fire rescue run confirmed saved: " + saveReceiptPath);
    }

    void RequestReset()
    {
        if (ros == null) return;
        resetPending = true;
        ros.SendServiceMessage<RosMessageTypes.Std.EmptyResponse>(
            "/victim_mission/reset",
            new RosMessageTypes.Std.EmptyRequest(),
            _ =>
            {
                rescuedCount = 0;
                missionState = "LISTENING";
                missionStatus = "mission reset";
                recoveryState = "normal";
                savedThisRun = false;
                completionPopup = false;
                resetPending = false;
                // Reset must restore physical trial state as well as ROS
                // counters. Reloading recreates the robot, victims, target,
                // trajectory and publishers at their authored start poses.
                reloadScenePending = true;
            });
    }

    void EnsureHudStyles()
    {
        if (hudStyle != null) return;
        hudStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 21,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft
        };
        hudStyle.normal.textColor = Color.white;

        hudShadowStyle = new GUIStyle(hudStyle);
        hudShadowStyle.normal.textColor = new Color(0f, 0f, 0f, 0.85f);
    }
}

public static class FireRescueRuntimeDisplayBootstrap
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void AttachToRobotRoot()
    {
        GameObject[] robotObjects;
        try
        {
            robotObjects = GameObject.FindGameObjectsWithTag("Robot");
        }
        catch (UnityException error)
        {
            Debug.LogError("Fire rescue display cannot find Robot tag: " + error.Message);
            return;
        }

        foreach (GameObject candidate in robotObjects)
        {
            if (candidate.GetComponent<CmdVelSubscriber>() == null) continue;
            if (candidate.GetComponent<FireRescueRuntimeDisplay>() == null)
            {
                candidate.AddComponent<FireRescueRuntimeDisplay>();
            }
            return;
        }
        Debug.LogWarning("FireRescueRuntimeDisplay could not find the Robot root.");
    }
}
