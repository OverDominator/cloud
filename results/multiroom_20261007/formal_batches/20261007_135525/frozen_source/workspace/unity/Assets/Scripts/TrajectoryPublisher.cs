using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;

[RequireComponent(typeof(LineRenderer))]
public class TrajectoryPublisher : MonoBehaviour
{
    [Header("ROS 发布")]
    public string topicName = "/trajectory";
    public string frameId = "map";
    public float publishRate = 10f;

    [Header("采样策略")]
    public float sampleInterval = 0.1f;   // 检查间隔(秒)
    public float minDistance = 0.05f;     // 最小记录距离(米),小于此值不记录
    public int maxPoints = 5000;          // 最大点数,超出后丢弃最早的点

    [Header("可视化")]
    public Color lineColor = new Color(0f, 1f, 0f, 0.9f);
    public float lineWidth = 0.05f;

    private ROSConnection ros;
    private LineRenderer lineRenderer;
    private readonly List<Vector3> points = new List<Vector3>();
    private float sampleTimer;
    private float publishTimer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PathMsg>(topicName);

        lineRenderer = GetComponent<LineRenderer>();
        ConfigureLineRenderer();

        // 记录起始点
        AddPoint(transform.position);
    }

    void ConfigureLineRenderer()
    {
        lineRenderer.positionCount = 0;
        lineRenderer.startWidth = lineWidth;
        lineRenderer.endWidth = lineWidth;
        lineRenderer.startColor = lineColor;
        lineRenderer.endColor = lineColor;
        lineRenderer.useWorldSpace = true;
        lineRenderer.numCornerVertices = 4;
        lineRenderer.numCapVertices = 4;

        // 尝试使用 Unlit/Color 着色器,缺失时回退到默认材质
        Shader unlit = Shader.Find("Unlit/Color");
        if (unlit != null)
        {
            lineRenderer.material = new Material(unlit);
            lineRenderer.material.color = lineColor;
        }
    }

    void Update()
    {
        // 1. 采样:按时间间隔检查,达到距离阈值才记录
        sampleTimer += Time.deltaTime;
        if (sampleTimer >= sampleInterval)
        {
            sampleTimer = 0f;
            TryAddPoint();
        }

        // 2. ROS 发布
        publishTimer += Time.deltaTime;
        if (publishTimer >= 1f / publishRate)
        {
            publishTimer = 0f;
            PublishPath();
        }
    }

    void TryAddPoint()
    {
        if (points.Count == 0)
        {
            AddPoint(transform.position);
            return;
        }

        Vector3 last = points[points.Count - 1];
        if (Vector3.Distance(last, transform.position) >= minDistance)
        {
            AddPoint(transform.position);
        }
    }

    void AddPoint(Vector3 position)
    {
        points.Add(position);

        // 超出上限则丢弃最早的点(滑动窗口)
        bool trimmed = false;
        while (points.Count > maxPoints)
        {
            points.RemoveAt(0);
            trimmed = true;
        }

        // 更新 LineRenderer
        if (trimmed)
        {
            lineRenderer.positionCount = points.Count;
            lineRenderer.SetPositions(points.ToArray());
        }
        else
        {
            lineRenderer.positionCount = points.Count;
            lineRenderer.SetPosition(points.Count - 1, position);
        }
    }

    void PublishPath()
    {
        if (points.Count == 0) return;

        TimeMsg stamp = GetRosTime();
        PoseStampedMsg[] poses = new PoseStampedMsg[points.Count];
        for (int i = 0; i < points.Count; i++)
        {
            // Unity 世界坐标 -> ROS FLU 坐标,与 OdomPublisher/TransformBroadcaster 保持一致
            poses[i] = new PoseStampedMsg
            {
                header = new HeaderMsg { stamp = stamp, frame_id = frameId },
                pose = new PoseMsg
                {
                    position = points[i].To<FLU>(),
                    orientation = new QuaternionMsg(0, 0, 0, 1)
                }
            };
        }

        PathMsg msg = new PathMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = frameId },
            poses = poses
        };
        ros.Publish(topicName, msg);
    }

    static TimeMsg GetRosTime()
    {
        long milliseconds = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        return new TimeMsg((uint)(milliseconds / 1000), (uint)((milliseconds % 1000) * 1_000_000));
    }
}
