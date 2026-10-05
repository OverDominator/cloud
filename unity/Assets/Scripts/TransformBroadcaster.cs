using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class TransformBroadcaster : MonoBehaviour
{
    public Transform lidarTransform;
    public Transform thermalCameraTransform;

    private ROSConnection ros;
    private float publishRate = 10f; // 降低 TF 带宽，保留足够平滑度
    private float timer;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>("/tf", 1);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= 1f / publishRate)
        {
            PublishTransforms();
            timer = 0;
        }
    }

    void PublishTransforms()
    {
        TFMessageMsg tfMsg = new TFMessageMsg();
        TimeMsg stamp = GetRosTime();
        tfMsg.transforms = new TransformStampedMsg[4];

        // Unity is the single source for the complete navigation TF chain.
        tfMsg.transforms[0] = CreateIdentityTF("map", "odom", stamp);
        tfMsg.transforms[1] = CreateWorldTF(transform, "odom", "vehicle", stamp);
        tfMsg.transforms[2] = CreateRelativeTF(transform, lidarTransform, "vehicle", "velodyne", stamp);
        tfMsg.transforms[3] = CreateRelativeTF(transform, thermalCameraTransform, "vehicle", "thermal_camera_link", stamp);

        ros.Publish("/tf", tfMsg);
    }

    TransformStampedMsg CreateIdentityTF(string parentFrame, string childFrame, TimeMsg stamp)
    {
        return new TransformStampedMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = parentFrame },
            child_frame_id = childFrame,
            transform = new TransformMsg
            {
                translation = new Vector3Msg(0, 0, 0),
                rotation = new QuaternionMsg(0, 0, 0, 1)
            }
        };
    }

    TransformStampedMsg CreateWorldTF(Transform child, string parentFrame, string childFrame, TimeMsg stamp)
    {
        return new TransformStampedMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = parentFrame },
            child_frame_id = childFrame,
            transform = new TransformMsg
            {
                translation = child.position.To<FLU>(),
                rotation = child.rotation.To<FLU>()
            }
        };
    }

    TransformStampedMsg CreateRelativeTF(Transform parent, Transform child, string parentFrame, string childFrame, TimeMsg stamp)
    {
        // 计算子物体相对于父物体的相对局部坐标
        Vector3 relativePos = parent.InverseTransformPoint(child.position);
        Quaternion relativeRot = Quaternion.Inverse(parent.rotation) * child.rotation;

        return new TransformStampedMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = parentFrame },
            child_frame_id = childFrame,
            transform = new TransformMsg
            {
                // 将 Unity 的 Vector3/Quaternion 转换为 ROS 标准的 FLU 坐标系格式
                translation = relativePos.To<FLU>(),
                rotation = relativeRot.To<FLU>()
            }
        };
    }

    static TimeMsg GetRosTime()
    {
        long milliseconds = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        return new TimeMsg((uint)(milliseconds / 1000), (uint)((milliseconds % 1000) * 1_000_000));
    }
}
