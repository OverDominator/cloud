using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;

public class RobotMovement : MonoBehaviour
{
    [Header("Manual demonstration")]
    public bool enableKeyboardControl = false;
    public float moveSpeed = 1.5f;
    public float rotateSpeed = 75.0f;
    [Tooltip("Hard cap for comparable human/FAR demonstrations, even when an open scene retains an old moveSpeed value.")]
    [Min(0.1f)] public float manualMaximumMoveSpeed = 1.0f;
    [Tooltip("Manual yaw-rate cap in rad/s; FAR normally uses at most about 0.9 rad/s.")]
    [Min(0.1f)] public float manualMaximumTurnRate = 0.8f;
    [Min(0.02f)] public float demonstrationSampleInterval = 0.05f;
    public bool autoSaveOnStop = true;

    [Header("Wheel visuals")]
    public Transform leftFrontWheel;
    public Transform leftRearWheel;
    public Transform rightFrontWheel;
    public Transform rightRearWheel;
    public float wheelRadius = 0.3f;

    public bool ManualModeActive => manualModeActive;
    public float ManualMoveInput => moveInput;
    public float ManualTurnInput => turnInput;
    public string LastSavedPath => lastSavedPath;

    struct ManualSample
    {
        public double elapsed;
        public string utc;
        public Vector3 unityPosition;
        public PointMsg rosPosition;
        public float unityYaw;
        public float linearVelocity;
        public float angularVelocity;
        public float move;
        public float turn;
        public string contact;
        public float contactNormalY;
    }

    Rigidbody body;
    CmdVelSubscriber rosDrive;
    bool manualModeActive;
    float moveInput;
    float turnInput;
    float nextSampleTime;
    double recordingStartedAt;
    DateTime recordingStartedUtc;
    readonly List<ManualSample> samples = new List<ManualSample>();
    int lastSavedSampleCount;
    string lastSavedPath = "not saved";
    string lastContact = "none";
    float lastContactNormalY;

    void Start()
    {
        body = GetComponent<Rigidbody>();
        rosDrive = GetComponent<CmdVelSubscriber>();
        SetManualMode(enableKeyboardControl, true);
    }

    void Update()
    {
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null) return;

        if (keyboard.mKey.wasPressedThisFrame)
            SetManualMode(!manualModeActive, false);
        if (keyboard.pKey.wasPressedThisFrame)
            SaveDemonstration("manual");
        if (keyboard.rKey.wasPressedThisFrame)
            ResetDemonstration();

        if (!manualModeActive)
        {
            moveInput = 0f;
            turnInput = 0f;
            return;
        }

        moveInput = 0f;
        turnInput = 0f;
        if (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed) moveInput += 1f;
        if (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed) moveInput -= 1f;
        if (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed) turnInput -= 1f;
        if (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed) turnInput += 1f;
        if (keyboard.spaceKey.isPressed)
        {
            moveInput = 0f;
            turnInput = 0f;
        }
    }

    void FixedUpdate()
    {
        if (!manualModeActive || body == null) return;

        float effectiveMoveSpeed = Mathf.Min(
            Mathf.Abs(moveSpeed), Mathf.Max(0.1f, manualMaximumMoveSpeed));
        float effectiveTurnRate = Mathf.Min(
            Mathf.Abs(rotateSpeed) * Mathf.Deg2Rad,
            Mathf.Max(0.1f, manualMaximumTurnRate));
        Vector3 desiredMove = transform.forward * (moveInput * effectiveMoveSpeed);
        body.linearVelocity = new Vector3(
            desiredMove.x, body.linearVelocity.y, desiredMove.z);
        body.angularVelocity = new Vector3(
            0f, turnInput * effectiveTurnRate, 0f);
        UpdateWheelVisuals();
        SampleDemonstration();
    }

    void SetManualMode(bool active, bool initial)
    {
        manualModeActive = active;
        enableKeyboardControl = active;
        moveInput = 0f;
        turnInput = 0f;

        // One component owns the Rigidbody at a time. Keeping /cmd_vel active
        // here made the old keyboard mode appear unresponsive and corrupted
        // the demonstration trajectory.
        if (rosDrive != null) rosDrive.enabled = !active;
        if (body != null)
        {
            body.linearVelocity = new Vector3(0f, body.linearVelocity.y, 0f);
            body.angularVelocity = Vector3.zero;
        }

        if (active)
        {
            if (initial || samples.Count == 0) ResetDemonstration();
            Debug.Log("Manual demonstration enabled: WASD/arrows drive, Space brakes, P saves, R resets, M returns to ROS.");
        }
        else
        {
            if (!initial && samples.Count > lastSavedSampleCount)
                SaveDemonstration("mode_exit");
            Debug.Log("Manual demonstration disabled; /cmd_vel control restored.");
        }
    }

    void ResetDemonstration()
    {
        samples.Clear();
        lastSavedSampleCount = 0;
        lastSavedPath = "not saved";
        recordingStartedAt = Time.realtimeSinceStartupAsDouble;
        recordingStartedUtc = DateTime.UtcNow;
        nextSampleTime = Time.unscaledTime;
        Debug.Log("Manual demonstration recording reset.");
    }

    void SampleDemonstration()
    {
        if (Time.unscaledTime < nextSampleTime) return;
        nextSampleTime = Time.unscaledTime + Mathf.Max(0.02f, demonstrationSampleInterval);

        Vector3 position = transform.position;
        samples.Add(new ManualSample
        {
            elapsed = Time.realtimeSinceStartupAsDouble - recordingStartedAt,
            utc = DateTime.UtcNow.ToString("O", CultureInfo.InvariantCulture),
            unityPosition = position,
            rosPosition = position.To<FLU>(),
            unityYaw = transform.eulerAngles.y,
            linearVelocity = Vector3.Dot(body.linearVelocity, transform.forward),
            angularVelocity = body.angularVelocity.y,
            move = moveInput,
            turn = turnInput,
            contact = lastContact,
            contactNormalY = lastContactNormalY,
        });
    }

    public void SaveDemonstration(string reason = "manual")
    {
        if (samples.Count == 0)
        {
            Debug.LogWarning("Manual demonstration has no samples to save.");
            return;
        }

        string directory = Path.Combine(
            Application.persistentDataPath, "ManualDemonstrations");
        Directory.CreateDirectory(directory);
        string scene = SanitizeFilePart(SceneManager.GetActiveScene().name);
        string filename = string.Format(
            CultureInfo.InvariantCulture,
            "manual_demo_{0}_{1}_{2}.csv",
            scene,
            DateTime.Now.ToString("yyyyMMdd_HHmmss_fff", CultureInfo.InvariantCulture),
            SanitizeFilePart(reason));
        string path = Path.Combine(directory, filename);

        var text = new StringBuilder(samples.Count * 180);
        text.AppendLine("# manual_navigation_demonstration_v1");
        text.AppendLine("# scene=" + SceneManager.GetActiveScene().name);
        text.AppendLine("# unity_version=" + Application.unityVersion);
        text.AppendLine("# started_utc=" + recordingStartedUtc.ToString("O", CultureInfo.InvariantCulture));
        float effectiveMoveSpeed = Mathf.Min(
            Mathf.Abs(moveSpeed), Mathf.Max(0.1f, manualMaximumMoveSpeed));
        float effectiveTurnRate = Mathf.Min(
            Mathf.Abs(rotateSpeed) * Mathf.Deg2Rad,
            Mathf.Max(0.1f, manualMaximumTurnRate));
        text.AppendLine("# configured_move_speed_mps=" + moveSpeed.ToString("R", CultureInfo.InvariantCulture));
        text.AppendLine("# configured_rotate_speed_degps=" + rotateSpeed.ToString("R", CultureInfo.InvariantCulture));
        text.AppendLine("# effective_move_speed_mps=" + effectiveMoveSpeed.ToString("R", CultureInfo.InvariantCulture));
        text.AppendLine("# effective_turn_rate_radps=" + effectiveTurnRate.ToString("R", CultureInfo.InvariantCulture));
        text.AppendLine("# fixed_delta_time=" + Time.fixedDeltaTime.ToString("R", CultureInfo.InvariantCulture));
        text.AppendLine("elapsed_s,utc,unity_x,unity_y,unity_z,ros_x,ros_y,ros_z,unity_yaw_deg,actual_linear_mps,actual_angular_radps,move_input,turn_input,contact,contact_normal_y");
        foreach (ManualSample sample in samples)
        {
            text.AppendFormat(CultureInfo.InvariantCulture,
                "{0:F4},{1},{2:F6},{3:F6},{4:F6},{5:F6},{6:F6},{7:F6},{8:F4},{9:F6},{10:F6},{11:F3},{12:F3},{13},{14:F4}\n",
                sample.elapsed,
                sample.utc,
                sample.unityPosition.x,
                sample.unityPosition.y,
                sample.unityPosition.z,
                sample.rosPosition.x,
                sample.rosPosition.y,
                sample.rosPosition.z,
                sample.unityYaw,
                sample.linearVelocity,
                sample.angularVelocity,
                sample.move,
                sample.turn,
                EscapeCsv(sample.contact),
                sample.contactNormalY);
        }

        File.WriteAllText(path, text.ToString(), Encoding.UTF8);
        lastSavedPath = path;
        lastSavedSampleCount = samples.Count;
        Debug.Log("Manual demonstration saved: " + path);
    }

    void UpdateWheelVisuals()
    {
        float effectiveMoveSpeed = Mathf.Min(
            Mathf.Abs(moveSpeed), Mathf.Max(0.1f, manualMaximumMoveSpeed));
        float effectiveTurnRate = Mathf.Min(
            Mathf.Abs(rotateSpeed) * Mathf.Deg2Rad,
            Mathf.Max(0.1f, manualMaximumTurnRate));
        float leftSpeed = moveInput * effectiveMoveSpeed -
            turnInput * effectiveTurnRate * wheelRadius;
        float rightSpeed = moveInput * effectiveMoveSpeed +
            turnInput * effectiveTurnRate * wheelRadius;
        float denominator = Mathf.Max(0.001f, 2f * Mathf.PI * wheelRadius);
        float leftAngle = leftSpeed * Time.fixedDeltaTime / denominator * 360f;
        float rightAngle = rightSpeed * Time.fixedDeltaTime / denominator * 360f;
        if (leftFrontWheel != null) leftFrontWheel.Rotate(leftAngle, 0f, 0f, Space.Self);
        if (leftRearWheel != null) leftRearWheel.Rotate(leftAngle, 0f, 0f, Space.Self);
        if (rightFrontWheel != null) rightFrontWheel.Rotate(rightAngle, 0f, 0f, Space.Self);
        if (rightRearWheel != null) rightRearWheel.Rotate(rightAngle, 0f, 0f, Space.Self);
    }

    void OnCollisionStay(Collision collision)
    {
        lastContact = collision.gameObject.name;
        lastContactNormalY = collision.contactCount > 0
            ? collision.GetContact(0).normal.y : 0f;
    }

    void OnCollisionExit(Collision collision)
    {
        if (lastContact == collision.gameObject.name)
        {
            lastContact = "none";
            lastContactNormalY = 0f;
        }
    }

    void OnDisable()
    {
        if (autoSaveOnStop && samples.Count > lastSavedSampleCount)
            SaveDemonstration("autosave");
    }

    void OnGUI()
    {
        if (!manualModeActive) return;
        Rect panel = new Rect(Mathf.Max(10f, Screen.width - 410f), 20f, 390f, 150f);
        GUI.Box(panel, "MANUAL DEMONSTRATION");
        GUI.Label(new Rect(panel.x + 12f, panel.y + 28f, 365f, 22f),
            "W/S: forward/reverse   A/D: turn   Space: brake");
        GUI.Label(new Rect(panel.x + 12f, panel.y + 52f, 365f, 22f),
            "M: ROS mode   P: save CSV   R: reset recording");
        GUI.Label(new Rect(panel.x + 12f, panel.y + 76f, 365f, 22f),
            "Samples: " + samples.Count + "   Last save: " + Path.GetFileName(lastSavedPath));
        if (GUI.Button(new Rect(panel.x + 12f, panel.y + 105f, 175f, 30f), "Save demonstration"))
            SaveDemonstration("button");
        if (GUI.Button(new Rect(panel.x + 202f, panel.y + 105f, 175f, 30f), "Reset recording"))
            ResetDemonstration();
    }

    static string SanitizeFilePart(string value)
    {
        foreach (char invalid in Path.GetInvalidFileNameChars())
            value = value.Replace(invalid, '_');
        return string.IsNullOrWhiteSpace(value) ? "unnamed" : value;
    }

    static string EscapeCsv(string value)
    {
        value = value ?? "";
        return "\"" + value.Replace("\"", "\"\"") + "\"";
    }
}
