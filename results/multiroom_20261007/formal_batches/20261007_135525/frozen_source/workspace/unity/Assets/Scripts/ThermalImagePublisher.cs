using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class ThermalImagePublisher : MonoBehaviour
{
    public Camera thermalCamera;
    public string topicName = "/thermal/image_raw";
    public string frameId = "thermal_camera_link";
    public string cameraInfoTopic = "/thermal/camera_info";
    // Thermal images are large ROS-TCP messages.  Keep the default low enough
    // that they cannot starve cmd/odom and mission-control traffic.
    public float publishRate = 0.5f;

    private float timeSinceLastPublish;
    private ROSConnection ros;
    private Texture2D readbackTexture;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        // Camera frames are snapshots, not a lossless event stream.  Keeping
        // twenty 170-KB frames queued only increases latency and can starve
        // odometry/status traffic; retain the newest frame only.
        ros.RegisterPublisher<ImageMsg>(topicName, 1);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopic, 1);

        if (thermalCamera != null)
        {
            // Only the driver's Main Camera owns scene audio.  Test scenes
            // historically serialized another listener on ThermalCamera,
            // producing an Editor warning every frame.
            AudioListener thermalListener = thermalCamera.GetComponent<AudioListener>();
            if (thermalListener != null) thermalListener.enabled = false;

            if (thermalCamera.targetTexture == null) return;
            CreateReadbackTexture();

            // A target-texture camera otherwise renders every game frame.
            // Render it manually only when a ROS image is due.
            thermalCamera.enabled = false;
        }
    }

    void Update()
    {
        if (publishRate <= 0f || ros == null || !ros.HasConnectionThread || ros.HasConnectionError) return;

        timeSinceLastPublish += Time.deltaTime;
        float interval = 1f / publishRate;
        if (timeSinceLastPublish < interval) return;

        timeSinceLastPublish %= interval;
        PublishThermalImage();
    }

    void PublishThermalImage()
    {
        if (thermalCamera == null || thermalCamera.targetTexture == null) return;
        // Exclude diagnostic rings and labels on the actual publishing camera,
        // even when it is untagged or created after the HUD's Start callback.
        int overlayLayer = LayerMask.NameToLayer("Ignore Raycast");
        if (overlayLayer >= 0) thermalCamera.cullingMask &= ~(1 << overlayLayer);

        if (readbackTexture == null ||
            readbackTexture.width != thermalCamera.targetTexture.width ||
            readbackTexture.height != thermalCamera.targetTexture.height)
        {
            CreateReadbackTexture();
        }

        SemanticThermalRender.Render(thermalCamera);

        RenderTexture previousActive = RenderTexture.active;
        RenderTexture.active = thermalCamera.targetTexture;
        readbackTexture.ReadPixels(
            new Rect(0, 0, readbackTexture.width, readbackTexture.height),
            0,
            0,
            false);
        RenderTexture.active = previousActive;

        int width = readbackTexture.width;
        int height = readbackTexture.height;
        int rowBytes = width * 3;
        var sourceData = readbackTexture.GetRawTextureData<byte>();
        byte[] flippedData = new byte[sourceData.Length];

        // Unity texture rows are bottom-up; sensor_msgs/Image is sent top-down.
        for (int y = 0; y < height; y++)
        {
            int sourceOffset = y * rowBytes;
            int destinationOffset = (height - 1 - y) * rowBytes;
            for (int index = 0; index < rowBytes; index++)
            {
                flippedData[destinationOffset + index] = sourceData[sourceOffset + index];
            }
        }

        long milliseconds = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        TimeMsg stamp = new TimeMsg(
            (uint)(milliseconds / 1000),
            (uint)((milliseconds % 1000) * 1_000_000));

        ImageMsg message = new ImageMsg
        {
            header = new HeaderMsg { frame_id = frameId, stamp = stamp },
            height = (uint)height,
            width = (uint)width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)rowBytes,
            data = flippedData
        };

        Matrix4x4 projection = thermalCamera.projectionMatrix;
        double fx = projection.m00 * width * 0.5;
        double fy = projection.m11 * height * 0.5;
        double cx = (1.0 - projection.m02) * width * 0.5;
        double cy = (1.0 + projection.m12) * height * 0.5;
        var info = new CameraInfoMsg {
            header = message.header, width = (uint)width, height = (uint)height,
            distortion_model = "plumb_bob", D = new double[5],
            K = new double[] { fx, 0, cx, 0, fy, cy, 0, 0, 1 },
            R = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
            P = new double[] { fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0 }
        };
        ros.Publish(cameraInfoTopic, info);
        ros.Publish(topicName, message);
    }

    void CreateReadbackTexture()
    {
        if (readbackTexture != null) Destroy(readbackTexture);

        readbackTexture = new Texture2D(
            thermalCamera.targetTexture.width,
            thermalCamera.targetTexture.height,
            TextureFormat.RGB24,
            false);
    }

    void OnDestroy()
    {
        if (readbackTexture != null) Destroy(readbackTexture);
    }
}
