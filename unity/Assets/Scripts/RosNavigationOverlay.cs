using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Visualization;

/// <summary>
/// A lightweight, read-only RViz replacement for the Unity Game/Scene view.
/// It never publishes commands and therefore cannot affect navigation.
/// </summary>
public class RosNavigationOverlay : MonoBehaviour
{
    [Header("ROS topics")]
    public string graphTopic = "/viz_graph_topic";
    public string globalPathTopic = "/viz_path_topic";
    public string localPathTopic = "/path";
    public string waypointTopic = "/way_point";
    public string goalTopic = "/goal_point";
    public string obstacleTopic = "/FAR_obs_debug";

    [Header("Display")]
    public bool showGraph = true;
    public bool showGlobalPath = true;
    public bool showLocalPath = true;
    public bool showTargets = true;
    public bool showObstacles = true;
    public float heightOffset = 0.10f;
    [Range(100, 5000)] public int maximumObstaclePoints = 1600;
    public float obstacleCrossSize = 0.055f;

    ROSConnection ros;
    GameObject graphObject;
    GameObject obstacleObject;
    GameObject globalPathObject;
    GameObject localPathObject;
    GameObject waypointObject;
    GameObject goalObject;
    Mesh graphMesh;
    Mesh obstacleMesh;
    LineRenderer globalPathLine;
    LineRenderer localPathLine;
    LineRenderer waypointMark;
    LineRenderer goalMark;
    Material graphMaterial;
    Material obstacleMaterial;
    Material globalPathMaterial;
    Material localPathMaterial;
    Material waypointMaterial;
    Material goalMaterial;
    GUIStyle titleStyle;
    GUIStyle labelStyle;
    float obstacleReceivedAt = float.NegativeInfinity;
    Rect panelRect = new Rect(12f, 170f, 235f, 190f);

    static readonly Color GraphColor = new Color(0f, 1f, 1f, 0.80f);
    static readonly Color GlobalPathColor = new Color(0.15f, 0.35f, 1f, 1f);
    static readonly Color LocalPathColor = new Color(1f, 0.48f, 0.05f, 1f);
    static readonly Color WaypointColor = new Color(1f, 0.9f, 0f, 1f);
    static readonly Color GoalColor = new Color(0.1f, 1f, 0.2f, 1f);
    static readonly Color ObstacleColor = new Color(1f, 0.05f, 0.05f, 0.85f);

    void Start()
    {
        BuildRenderers();
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<MarkerArrayMsg>(graphTopic, OnGraph);
        ros.Subscribe<MarkerMsg>(globalPathTopic, OnGlobalPath);
        ros.Subscribe<PathMsg>(localPathTopic, OnLocalPath);
        ros.Subscribe<PointStampedMsg>(waypointTopic, msg => OnTarget(msg, waypointMark));
        ros.Subscribe<PointStampedMsg>(goalTopic, msg => OnTarget(msg, goalMark));
        ros.Subscribe<PointCloud2Msg>(obstacleTopic, OnObstacles);
        ApplyVisibility();
        Debug.Log("ROS navigation overlay enabled (F8 toggles the panel).");
    }

    void BuildRenderers()
    {
        graphMaterial = MakeMaterial(GraphColor);
        obstacleMaterial = MakeMaterial(ObstacleColor);
        globalPathMaterial = MakeMaterial(GlobalPathColor);
        localPathMaterial = MakeMaterial(LocalPathColor);
        waypointMaterial = MakeMaterial(WaypointColor);
        goalMaterial = MakeMaterial(GoalColor);

        graphObject = CreateMeshObject("ROS Graph", graphMaterial, out graphMesh);
        obstacleObject = CreateMeshObject("ROS Obstacles", obstacleMaterial, out obstacleMesh);
        globalPathLine = CreateLine("ROS FAR Path", globalPathMaterial, 0.09f);
        globalPathObject = globalPathLine.gameObject;
        localPathLine = CreateLine("ROS Local Path", localPathMaterial, 0.065f);
        localPathObject = localPathLine.gameObject;
        waypointMark = CreateLine("ROS Waypoint", waypointMaterial, 0.07f);
        waypointObject = waypointMark.gameObject;
        goalMark = CreateLine("ROS Goal", goalMaterial, 0.09f);
        goalObject = goalMark.gameObject;
    }

    GameObject CreateMeshObject(string objectName, Material material, out Mesh mesh)
    {
        GameObject obj = new GameObject(objectName);
        AssignVisualizationLayer(obj);
        // Mesh vertices are already expressed in Unity world coordinates.
        // Keep the renderer at the scene root so robot motion is not applied twice.
        obj.transform.SetParent(null, false);
        MeshFilter filter = obj.AddComponent<MeshFilter>();
        MeshRenderer renderer = obj.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = material;
        mesh = new Mesh { name = objectName + " Mesh" };
        mesh.MarkDynamic();
        filter.sharedMesh = mesh;
        return obj;
    }

    LineRenderer CreateLine(string objectName, Material material, float width)
    {
        GameObject obj = new GameObject(objectName);
        AssignVisualizationLayer(obj);
        obj.transform.SetParent(transform, false);
        LineRenderer line = obj.AddComponent<LineRenderer>();
        line.sharedMaterial = material;
        line.startColor = material.color;
        line.endColor = material.color;
        line.startWidth = width;
        line.endWidth = width;
        line.numCornerVertices = 3;
        line.numCapVertices = 2;
        line.useWorldSpace = true;
        line.positionCount = 0;
        return line;
    }

    static void AssignVisualizationLayer(GameObject obj)
    {
        // FireRescueRuntimeDisplay removes this layer from every ThermalCamera
        // culling mask.  Keeping debug geometry here prevents RViz-like lines
        // from contaminating victim detection images while remaining visible
        // to the normal Game and Scene cameras.
        int visualizationLayer = LayerMask.NameToLayer("Ignore Raycast");
        if (visualizationLayer >= 0) obj.layer = visualizationLayer;
    }

    static Material MakeMaterial(Color color)
    {
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null) shader = Shader.Find("Sprites/Default");
        if (shader == null) shader = Shader.Find("Unlit/Color");
        Material material = new Material(shader) { color = color };
        material.renderQueue = 3100;
        return material;
    }

    void OnGraph(MarkerArrayMsg message)
    {
        if (message == null || message.markers == null) return;
        List<Vector3> vertices = new List<Vector3>();
        foreach (MarkerMsg marker in message.markers)
        {
            if (marker == null || marker.action == MarkerMsg.DELETE || marker.action == MarkerMsg.DELETEALL || marker.points == null) continue;
            bool local = IsLocalFrame(marker.header != null ? marker.header.frame_id : null);
            Vector3 poseOffset = marker.pose != null && marker.pose.position != null
                ? RosPoint(marker.pose.position, local) - RosPoint(new PointMsg(), local)
                : Vector3.zero;

            if (marker.type == MarkerMsg.LINE_LIST)
            {
                for (int i = 0; i + 1 < marker.points.Length; i += 2)
                {
                    vertices.Add(RosPoint(marker.points[i], local) + poseOffset);
                    vertices.Add(RosPoint(marker.points[i + 1], local) + poseOffset);
                }
            }
            else if (marker.type == MarkerMsg.LINE_STRIP)
            {
                for (int i = 0; i + 1 < marker.points.Length; i++)
                {
                    vertices.Add(RosPoint(marker.points[i], local) + poseOffset);
                    vertices.Add(RosPoint(marker.points[i + 1], local) + poseOffset);
                }
            }
            else if (marker.type == MarkerMsg.SPHERE_LIST || marker.type == MarkerMsg.CUBE_LIST || marker.type == MarkerMsg.POINTS)
            {
                float size = marker.scale == null ? 0.08f : Mathf.Clamp((float)Math.Max(marker.scale.x, marker.scale.y) * 0.25f, 0.035f, 0.16f);
                foreach (PointMsg point in marker.points) AddCross(vertices, RosPoint(point, local) + poseOffset, size);
            }
        }
        SetLineMesh(graphMesh, vertices);
    }

    void OnGlobalPath(MarkerMsg marker)
    {
        if (marker == null || marker.points == null || marker.action == MarkerMsg.DELETE || marker.action == MarkerMsg.DELETEALL)
        {
            globalPathLine.positionCount = 0;
            return;
        }
        bool local = IsLocalFrame(marker.header != null ? marker.header.frame_id : null);
        Vector3[] points = new Vector3[marker.points.Length];
        for (int i = 0; i < points.Length; i++) points[i] = RosPoint(marker.points[i], local);
        globalPathLine.positionCount = points.Length;
        globalPathLine.SetPositions(points);
    }

    void OnLocalPath(PathMsg path)
    {
        if (path == null || path.poses == null)
        {
            localPathLine.positionCount = 0;
            return;
        }
        bool local = IsLocalFrame(path.header != null ? path.header.frame_id : null);
        Vector3[] points = new Vector3[path.poses.Length];
        for (int i = 0; i < points.Length; i++) points[i] = RosPoint(path.poses[i].pose.position, local);
        localPathLine.positionCount = points.Length;
        localPathLine.SetPositions(points);
    }

    void OnTarget(PointStampedMsg target, LineRenderer mark)
    {
        if (target == null || target.point == null) return;
        bool local = IsLocalFrame(target.header != null ? target.header.frame_id : null);
        Vector3 center = RosPoint(target.point, local);
        const int segments = 32;
        Vector3[] points = new Vector3[segments + 1];
        float radius = mark == goalMark ? 0.38f : 0.24f;
        for (int i = 0; i <= segments; i++)
        {
            float angle = i * Mathf.PI * 2f / segments;
            points[i] = center + new Vector3(Mathf.Cos(angle) * radius, 0.02f, Mathf.Sin(angle) * radius);
        }
        mark.positionCount = points.Length;
        mark.SetPositions(points);
    }

    void OnObstacles(PointCloud2Msg cloud)
    {
        if (cloud == null || cloud.data == null || cloud.fields == null || cloud.point_step == 0) return;
        string frame = cloud.header?.frame_id?.TrimStart('/');
        TFFrame cloudTransform = TFFrame.identity;
        if (frame != "map")
        {
            var stream = string.IsNullOrEmpty(frame) ? null :
                TFSystem.GetOrCreateInstance().GetTransformStream(frame);
            long stamp = cloud.header == null ? 0 : cloud.header.stamp.ToLongTime();
            // Never guess a frame or extrapolate using the moving robot's pose.
            if (stream == null || !stream.IsTimeStable(stamp))
            {
                obstacleMesh.Clear();
                return;
            }
            cloudTransform = stream.GetWorldTF(stamp);
        }
        int xOffset = FindField(cloud, "x");
        int yOffset = FindField(cloud, "y");
        int zOffset = FindField(cloud, "z");
        if (xOffset < 0 || yOffset < 0 || zOffset < 0) return;

        int total = Math.Min((int)(cloud.width * cloud.height), cloud.data.Length / (int)cloud.point_step);
        int stride = Math.Max(1, (int)Math.Ceiling(total / (double)maximumObstaclePoints));
        List<Vector3> vertices = new List<Vector3>(Math.Min(total, maximumObstaclePoints) * 6);
        for (int i = 0; i < total; i += stride)
        {
            int row = i / Math.Max(1, (int)cloud.width);
            int column = i % Math.Max(1, (int)cloud.width);
            int baseOffset = row * (int)cloud.row_step + column * (int)cloud.point_step;
            float x = ReadFloat(cloud.data, baseOffset + xOffset, cloud.is_bigendian);
            float y = ReadFloat(cloud.data, baseOffset + yOffset, cloud.is_bigendian);
            float z = ReadFloat(cloud.data, baseOffset + zOffset, cloud.is_bigendian);
            if (!float.IsFinite(x) || !float.IsFinite(y) || !float.IsFinite(z)) continue;
            Vector3 point = cloudTransform.TransformPoint(new Vector3(-y, z, x));
            AddCross(vertices, point + Vector3.up * heightOffset, obstacleCrossSize);
        }
        SetLineMesh(obstacleMesh, vertices);
        obstacleReceivedAt = Time.unscaledTime;
    }

    void LateUpdate()
    {
        // Do not leave the last obstacle frame visible after a disconnect/reset.
        if (obstacleMesh != null && obstacleMesh.vertexCount > 0 &&
            Time.unscaledTime - obstacleReceivedAt > 2f)
            obstacleMesh.Clear();
    }

    int FindField(PointCloud2Msg cloud, string name)
    {
        foreach (PointFieldMsg field in cloud.fields)
            if (field != null && field.name == name && field.datatype == PointFieldMsg.FLOAT32) return (int)field.offset;
        return -1;
    }

    static float ReadFloat(byte[] data, int offset, bool bigEndian)
    {
        if (offset < 0 || offset + 4 > data.Length) return float.NaN;
        if (!bigEndian) return BitConverter.ToSingle(data, offset);
        byte[] bytes = { data[offset + 3], data[offset + 2], data[offset + 1], data[offset] };
        return BitConverter.ToSingle(bytes, 0);
    }

    Vector3 RosPoint(PointMsg point, bool local)
    {
        if (point == null) return Vector3.zero;
        return RosPoint((float)point.x, (float)point.y, (float)point.z, local);
    }

    Vector3 RosPoint(float x, float y, float z, bool local)
    {
        Vector3 unity = new Vector3(-y, z + heightOffset, x);
        return local ? transform.TransformPoint(unity) : unity;
    }

    static bool IsLocalFrame(string frame)
    {
        if (string.IsNullOrWhiteSpace(frame)) return false;
        frame = frame.TrimStart('/').ToLowerInvariant();
        return frame == "vehicle" || frame == "base_link" || frame == "base_footprint" || frame.Contains("sensor");
    }

    static void AddCross(List<Vector3> vertices, Vector3 center, float size)
    {
        vertices.Add(center - Vector3.right * size); vertices.Add(center + Vector3.right * size);
        vertices.Add(center - Vector3.forward * size); vertices.Add(center + Vector3.forward * size);
        vertices.Add(center - Vector3.up * size); vertices.Add(center + Vector3.up * size);
    }

    static void SetLineMesh(Mesh mesh, List<Vector3> vertices)
    {
        mesh.Clear();
        if (vertices.Count == 0) return;
        mesh.SetVertices(vertices);
        int[] indices = new int[vertices.Count];
        for (int i = 0; i < indices.Length; i++) indices[i] = i;
        mesh.SetIndices(indices, MeshTopology.Lines, 0, false);
        mesh.RecalculateBounds();
    }

    void OnGUI()
    {
        if (panelRect.height <= 0f) return;
        if (titleStyle == null)
        {
            titleStyle = new GUIStyle(GUI.skin.label) { fontSize = 15, fontStyle = FontStyle.Bold };
            titleStyle.normal.textColor = Color.white;
            labelStyle = new GUIStyle(GUI.skin.label) { fontSize = 12 };
            labelStyle.normal.textColor = new Color(0.88f, 0.92f, 1f);
        }
        GUI.Box(panelRect, GUIContent.none);
        GUILayout.BeginArea(new Rect(panelRect.x + 10f, panelRect.y + 7f, panelRect.width - 20f, panelRect.height - 12f));
        GUILayout.Label("ROS 导航可视化", titleStyle);
        bool changed = false;
        changed |= Toggle(ref showGraph, "青色  GlobalGraph");
        changed |= Toggle(ref showGlobalPath, "蓝色  FAR 全局路径");
        changed |= Toggle(ref showLocalPath, "橙色  局部路径");
        changed |= Toggle(ref showTargets, "黄色路点 / 绿色终点");
        changed |= Toggle(ref showObstacles, "红色  FAR 障碍点");
        if (changed) ApplyVisibility();
        GUILayout.EndArea();
    }

    bool Toggle(ref bool value, string text)
    {
        bool next = GUILayout.Toggle(value, text, labelStyle);
        bool changed = next != value;
        value = next;
        return changed;
    }

    void ApplyVisibility()
    {
        if (graphObject != null) graphObject.SetActive(showGraph);
        if (globalPathObject != null) globalPathObject.SetActive(showGlobalPath);
        if (localPathObject != null) localPathObject.SetActive(showLocalPath);
        if (waypointObject != null) waypointObject.SetActive(showTargets);
        if (goalObject != null) goalObject.SetActive(showTargets);
        if (obstacleObject != null) obstacleObject.SetActive(showObstacles);
    }

    void OnDestroy()
    {
        if (graphObject != null) Destroy(graphObject);
        if (obstacleObject != null) Destroy(obstacleObject);
        if (globalPathObject != null) Destroy(globalPathObject);
        if (localPathObject != null) Destroy(localPathObject);
        if (waypointObject != null) Destroy(waypointObject);
        if (goalObject != null) Destroy(goalObject);
        Destroy(graphMaterial);
        Destroy(obstacleMaterial);
        Destroy(globalPathMaterial);
        Destroy(localPathMaterial);
        Destroy(waypointMaterial);
        Destroy(goalMaterial);
    }
}

public static class RosNavigationOverlayBootstrap
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Attach()
    {
        CmdVelSubscriber robot = UnityEngine.Object.FindFirstObjectByType<CmdVelSubscriber>();
        if (robot == null)
        {
            Debug.LogWarning("ROS navigation overlay could not find CmdVelSubscriber.");
            return;
        }
        if (robot.GetComponent<RosNavigationOverlay>() == null) robot.gameObject.AddComponent<RosNavigationOverlay>();
    }
}
