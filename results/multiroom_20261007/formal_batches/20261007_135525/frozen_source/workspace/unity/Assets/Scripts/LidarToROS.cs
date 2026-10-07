using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

[RequireComponent(typeof(LidarSensor))]
public class LidarToROS : MonoBehaviour
{
    public string topicName = "/velodyne_points";
    public string frameId = "velodyne";
    public string registeredTopicName = "/registered_scan";
    public string registeredFrameId = "map";
    public bool publishLocalCloud = false;
    public bool publishRegisteredCloud = true;
    // Registered point clouds are also relatively large; 3 Hz is sufficient
    // for FAR while leaving TCP bandwidth for control and odometry.
    public float publishRateHz = 1f;
    public float defaultIntensity = 255f;

    private ROSConnection ros;
    private LidarSensor lidarSensor;
    private float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // A scan is a latest-state snapshot.  Never accumulate stale point
        // clouds behind the socket; retaining one frame bounds both memory and
        // control-topic latency during a short ROS-TCP slowdown.
        if (publishLocalCloud) ros.RegisterPublisher<PointCloud2Msg>(topicName, 1);
        if (publishRegisteredCloud) ros.RegisterPublisher<PointCloud2Msg>(registeredTopicName, 1);
        lidarSensor = GetComponent<LidarSensor>();
    }

    void Update()
    {
        if (publishRateHz <= 0f || ros == null || !ros.HasConnectionThread || ros.HasConnectionError) return;
        timer += Time.deltaTime;
        float interval = 1f / publishRateHz;
        if (timer < interval) return;
        timer %= interval;
        PublishPointCloud();
    }

    void PublishPointCloud()
    {
        List<Vector3> worldPoints = lidarSensor.pointCloudData;
        if (worldPoints == null || worldPoints.Count == 0) return;

        const int pointStep = 16;
        int pointCount = worldPoints.Count;
        byte[] localData = publishLocalCloud
            ? new byte[pointCount * pointStep] : null;
        byte[] registeredData = publishRegisteredCloud
            ? new byte[pointCount * pointStep] : null;

        for (int i = 0; i < pointCount; i++)
        {
            // RaycastHit.point is in world space; PointCloud2 data must be
            // expressed relative to the frame named by header.frame_id.
            if (publishLocalCloud)
            {
                Vector3 point = transform.InverseTransformPoint(worldPoints[i]);
                WritePoint(localData, i * pointStep, point);
            }
            if (publishRegisteredCloud)
                WritePoint(registeredData, i * pointStep, worldPoints[i]);
        }

        TimeMsg stamp = GetRosTime();
        if (publishLocalCloud)
            ros.Publish(topicName, CreateMessage(frameId, stamp, pointCount, localData));
        if (publishRegisteredCloud)
            ros.Publish(registeredTopicName, CreateMessage(registeredFrameId, stamp, pointCount, registeredData));
    }

    PointCloud2Msg CreateMessage(string cloudFrameId, TimeMsg stamp, int pointCount, byte[] data)
    {
        const int pointStep = 16;
        return new PointCloud2Msg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = cloudFrameId },
            height = 1,
            width = (uint)pointCount,
            fields = new[]
            {
                new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("intensity", 12, PointFieldMsg.FLOAT32, 1)
            },
            is_bigendian = false,
            point_step = pointStep,
            row_step = (uint)(pointCount * pointStep),
            data = data,
            is_dense = true
        };
    }

    void WritePoint(byte[] destination, int offset, Vector3 unityPoint)
    {
        // Unity (right, up, forward) -> ROS FLU (forward, left, up).
        WriteFloat(destination, offset, unityPoint.z);
        WriteFloat(destination, offset + 4, -unityPoint.x);
        WriteFloat(destination, offset + 8, unityPoint.y);
        WriteFloat(destination, offset + 12, defaultIntensity);
    }

    static void WriteFloat(byte[] destination, int offset, float value)
    {
        Buffer.BlockCopy(BitConverter.GetBytes(value), 0, destination, offset, sizeof(float));
    }

    static TimeMsg GetRosTime()
    {
        long milliseconds = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        return new TimeMsg((uint)(milliseconds / 1000), (uint)((milliseconds % 1000) * 1_000_000));
    }
}
