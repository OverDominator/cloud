using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;

public class OdomPublisher : MonoBehaviour
{
    public string topicName = "/odom";
    public string frameId = "odom";
    public string childFrameId = "vehicle";
    public float publishRate = 10f;

    private ROSConnection ros;
    private float timer;
    private double previousSampleTime;
    private Vector3 previousPosition;
    private Quaternion previousRotation;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // Odometry is latest-state data.  A deep queue only replays obsolete
        // poses after a network stall and delays navigation recovery.
        ros.RegisterPublisher<OdometryMsg>(topicName, 1);
        previousPosition = transform.position;
        previousRotation = transform.rotation;
        previousSampleTime = Time.timeAsDouble;
    }

    void Update()
    {
        if (publishRate <= 0f) return;
        // During ROS-TCP reconnect the endpoint may not have received the
        // publisher registration yet.  Suppress odometry for that short
        // window instead of sending an unregistered /odom message.
        if (ros == null || !ros.HasConnectionThread || ros.HasConnectionError)
            return;
        timer += Time.deltaTime;
        float interval = 1f / publishRate;
        if (timer < interval) return;
        timer %= interval;
        // The rate limiter retains its remainder; it is NOT the elapsed time
        // between the two poses used to measure velocity.
        double sampleTime = Time.timeAsDouble;
        float elapsed = (float)(sampleTime - previousSampleTime);
        if (elapsed <= 0f) return;
        previousSampleTime = sampleTime;
        PublishOdom(elapsed);
    }

    void PublishOdom(float elapsed)
    {
        Vector3 position = transform.position;
        Quaternion rotation = transform.rotation;
        Vector3 worldLinearVelocity = (position - previousPosition) / elapsed;

        Quaternion delta = Quaternion.Inverse(previousRotation) * rotation;
        delta.ToAngleAxis(out float angleDegrees, out Vector3 axis);
        if (angleDegrees > 180f) angleDegrees -= 360f;
        Vector3 localAngularVelocity = float.IsNaN(axis.x)
            ? Vector3.zero
            : axis.normalized * angleDegrees * Mathf.Deg2Rad / elapsed;

        previousPosition = position;
        previousRotation = rotation;

        OdometryMsg msg = new OdometryMsg
        {
            header = new HeaderMsg { stamp = GetRosTime(), frame_id = frameId },
            child_frame_id = childFrameId,
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = position.To<FLU>(),
                    orientation = rotation.To<FLU>()
                }
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = transform.InverseTransformDirection(worldLinearVelocity).To<FLU>(),
                    // Angular velocity is an axial vector. RUF -> FLU changes
                    // handedness, so ordinary vector conversion reverses the
                    // feedback sign relative to the published pose and cmd_vel.
                    angular = Vector3<FLU>.FromUnityAngularVelocity(localAngularVelocity)
                }
            }
        };
        ros.Publish(topicName, msg);
    }

    static TimeMsg GetRosTime()
    {
        long milliseconds = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        return new TimeMsg((uint)(milliseconds / 1000), (uint)((milliseconds % 1000) * 1_000_000));
    }
}
