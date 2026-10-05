using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

[DisallowMultipleComponent]
public class VictimCallSensor : MonoBehaviour
{
    public string victimTag = "Victim";
    public string detectedTopic = "/victim_call/detected";
    public string bearingTopic = "/victim_call/bearing";
    public string strengthTopic = "/victim_call/strength";
    public string precisePositionTopic = "/victim_call/precise_position";
    public string remainingTopic = "/victim_call/remaining";
    public string totalTopic = "/victim_call/total";
    public string nearestDistanceTopic = "/victim_call/nearest_distance";
    public string inRescueRangeTopic = "/victim_call/in_rescue_range";
    public string missionRescuedCountTopic = "/victim_mission/rescued_count";
    public string frameId = "vehicle";
    // Directional distress call range and physical rescue decision range.
    // Both are visualized around every victim at runtime.
    public float maximumHearingDistance = 35f;
    public float rescueDistance = 3f;
    public float publishRate = 2f;
    public int acousticSeed = 27001;
    public float acousticFarError = 5f;
    public float acousticNearError = 0.35f;
    private Transform lockedCaller;
    private System.Random acousticRandom;
    private Vector2 acousticBias;
    private Vector2 acousticBiasTarget;
    private float nextAcousticChange;

    private readonly HashSet<int> rescuedVictims = new HashSet<int>();
    private readonly Dictionary<int, float> rescueStarted = new Dictionary<int, float>();
    private readonly string episodeId = Guid.NewGuid().ToString("N");
    public float rescueHoldSeconds = 1f;
    const string SnapshotTopic = "/victim_call/scene_state";
    private ROSConnection ros;
    private Transform sensingOrigin;
    private float elapsed;
    private int lastMissionRescuedCount;
    private bool receivedInitialRescueCount;
    private int episodeVictimTotal;

    void Start()
    {
        // This topic carries a simulated noisy estimate, never ground truth.
        precisePositionTopic = "/victim_call/estimated_position";
        acousticRandom = new System.Random(acousticSeed);
        ros = ROSConnection.GetOrCreateInstance();
        LidarSensor lidar = GetComponentInChildren<LidarSensor>(true);
        sensingOrigin = lidar != null ? lidar.transform : transform;
        ros.RegisterPublisher<BoolMsg>(detectedTopic, 1);
        ros.RegisterPublisher<Vector3StampedMsg>(bearingTopic, 1);
        ros.RegisterPublisher<Float32Msg>(strengthTopic, 1);
        ros.RegisterPublisher<PointStampedMsg>(precisePositionTopic, 1);
        ros.RegisterPublisher<Int32Msg>(remainingTopic, 1);
        ros.RegisterPublisher<Int32Msg>(totalTopic, 1);
        ros.RegisterPublisher<Float32Msg>(nearestDistanceTopic, 1);
        ros.RegisterPublisher<BoolMsg>(inRescueRangeTopic, 1);
        ros.RegisterPublisher<StringMsg>(SnapshotTopic, 1);
        episodeVictimTotal = CountVictimRoots();
        RefreshVictimVisualizers();
    }

    void Update()
    {
        if (publishRate <= 0f) return;
        elapsed += Time.deltaTime;
        float interval = 1f / publishRate;
        if (elapsed < interval) return;
        elapsed %= interval;
        PublishNearestCall();
    }

    void PublishNearestCall()
    {
        GameObject[] victims;
        try
        {
            victims = GameObject.FindGameObjectsWithTag(victimTag);
        }
        catch (UnityException error)
        {
            Debug.LogError($"Victim call sensor cannot find tag '{victimTag}': {error.Message}");
            PublishNoCall();
            return;
        }

        Transform nearest = null;
        float nearestDistance = float.PositiveInfinity;
        float nearestPhysicalDistance = float.PositiveInfinity;
        bool insideRescueTrigger = false;
        int remainingVictims = 0;
        HashSet<int> visitedVictimRoots = new HashSet<int>();
        foreach (GameObject victim in victims)
        {
            Transform victimRoot = FindTaggedVictimRoot(victim.transform);
            int victimId = victimRoot.gameObject.GetInstanceID();
            if (!visitedVictimRoots.Add(victimId)) continue;
            if (!victimRoot.gameObject.activeInHierarchy || rescuedVictims.Contains(victimId)) continue;

            float distance = PlanarDistance(sensingOrigin.position, victimRoot.position);
            nearestPhysicalDistance = Mathf.Min(nearestPhysicalDistance, distance);
            VictimRangeVisualizer range =
                victimRoot.GetComponent<VictimRangeVisualizer>();
            if (range != null && range.IsRobotInside)
            {
                if (!rescueStarted.ContainsKey(victimId)) rescueStarted[victimId] = Time.time;
                if (Time.time - rescueStarted[victimId] >= Mathf.Max(0f, rescueHoldSeconds))
                {
                    rescuedVictims.Add(victimId);
                    rescueStarted.Remove(victimId);
                    range.SetRescued(true);
                    RemoveRescuedVictim(victimRoot.gameObject);
                    continue;
                }
            }
            else rescueStarted.Remove(victimId);
            remainingVictims++;
            insideRescueTrigger |= range != null && range.IsRobotInside;
            // A distress call is received only after the robot physically
            // enters this victim's Unity call trigger.  This keeps call and
            // rescue decisions on the same authoritative geometry.
            bool insideCallTrigger = range != null && range.IsRobotInsideCall;
            if (insideCallTrigger && (victimRoot == lockedCaller ||
                ((nearest == null || nearest != lockedCaller) && distance < nearestDistance)))
            {
                nearest = victimRoot;
                nearestDistance = distance;
            }
        }

        ros.Publish(totalTopic, new Int32Msg(episodeVictimTotal));
        // Repeated authoritative snapshot tolerates a dropped message and
        // identifies the exact rescued objects within this Unity episode.
        // The episode total must stay immutable after rescued victim objects
        // are destroyed.  Publishing the number of currently active roots
        // shrank 2 to 1 after the first rescue, which made ROS interpret 1/2
        // as 1/1 and stop the mission prematurely.
        ros.Publish(SnapshotTopic, new StringMsg(episodeId + "|" + episodeVictimTotal
            + "|" + string.Join(",", rescuedVictims) + "|"
            + (nearest != null ? nearest.gameObject.GetInstanceID().ToString() : "")));
        PublishRescueProximity(nearestPhysicalDistance, insideRescueTrigger);

        if (nearest == null)
        {
            PublishNoCall(
                remainingVictims,
                nearestPhysicalDistance,
                insideRescueTrigger);
            return;
        }

        if (lockedCaller != nearest)
        {
            lockedCaller = nearest;
            nextAcousticChange = 0f;
            acousticBias = Vector2.zero;
        }
        if (Time.time >= nextAcousticChange)
        {
            double angle = acousticRandom.NextDouble() * Math.PI * 2;
            float magnitude = Mathf.Sqrt((float)acousticRandom.NextDouble());
            acousticBiasTarget = new Vector2((float)Math.Cos(angle), (float)Math.Sin(angle)) * magnitude;
            nextAcousticChange = Time.time + 4f;
        }
        acousticBias = Vector2.Lerp(acousticBias, acousticBiasTarget,
            1f - Mathf.Exp(-1f / Mathf.Max(0.1f, publishRate)));
        float fraction = Mathf.Clamp01(nearestDistance / Mathf.Max(1f, maximumHearingDistance));
        float errorRadius = Mathf.Lerp(acousticNearError, acousticFarError, fraction * fraction);
        Vector3 estimatedPosition = nearest.position + new Vector3(acousticBias.x, 0, acousticBias.y) * errorRadius;
        Vector3 worldDirection = (estimatedPosition - sensingOrigin.position).normalized;
        Vector3 localDirection = transform.InverseTransformDirection(worldDirection);
        TimeMsg stamp = CurrentRosTime();
        ros.Publish(detectedTopic, new BoolMsg(true));
        ros.Publish(
            precisePositionTopic,
            new PointStampedMsg
            {
                header = new HeaderMsg { stamp = stamp, frame_id = "map" },
                point = estimatedPosition.To<FLU>()
            });
        ros.Publish(
            bearingTopic,
            new Vector3StampedMsg
            {
                header = new HeaderMsg { stamp = stamp, frame_id = frameId },
                // Unity local (right, up, forward) -> ROS FLU (forward, left, up).
                vector = new Vector3Msg(
                    localDirection.z,
                    -localDirection.x,
                    localDirection.y)
            });
        float strength = Mathf.Clamp01(1f - PlanarDistance(sensingOrigin.position, estimatedPosition) / maximumHearingDistance);
        ros.Publish(strengthTopic, new Float32Msg(strength));
        ros.Publish(remainingTopic, new Int32Msg(remainingVictims));
    }

    void PublishNoCall(
        int remainingVictims = 0,
        float nearestPhysicalDistance = float.PositiveInfinity,
        bool insideRescueTrigger = false)
    {
        TimeMsg stamp = CurrentRosTime();
        ros.Publish(detectedTopic, new BoolMsg(false));
        ros.Publish(
            bearingTopic,
            new Vector3StampedMsg
            {
                header = new HeaderMsg { stamp = stamp, frame_id = frameId },
                vector = new Vector3Msg(0, 0, 0)
            });
        ros.Publish(strengthTopic, new Float32Msg(0));
        ros.Publish(remainingTopic, new Int32Msg(remainingVictims));
        ros.Publish(totalTopic, new Int32Msg(episodeVictimTotal));
        PublishRescueProximity(nearestPhysicalDistance, insideRescueTrigger);
    }

    void PublishRescueProximity(float distance, bool insideRescueTrigger)
    {
        bool valid = !float.IsInfinity(distance) && !float.IsNaN(distance);
        ros.Publish(nearestDistanceTopic,
            new Float32Msg(valid ? distance : -1f));
        ros.Publish(inRescueRangeTopic,
            new BoolMsg(insideRescueTrigger));
    }

    int CountVictimRoots()
    {
        try
        {
            HashSet<int> roots = new HashSet<int>();
            foreach (GameObject victim in GameObject.FindGameObjectsWithTag(victimTag))
            {
                roots.Add(FindTaggedVictimRoot(victim.transform).gameObject.GetInstanceID());
            }
            return roots.Count;
        }
        catch (UnityException)
        {
            return 0;
        }
    }

    void OnMissionRescuedCount(Int32Msg message)
    {
        int rescuedCount = Math.Max(0, message.data);
        // A latched count received immediately after Unity reconnects is not a
        // new rescue event. Replaying it used to mark distant victims rescued.
        if (!receivedInitialRescueCount)
        {
            receivedInitialRescueCount = true;
            lastMissionRescuedCount = rescuedCount;
            RefreshVictimVisualizers();
            return;
        }
        if (rescuedCount < lastMissionRescuedCount)
        {
            // ROS mission reset: make every simulated victim callable again.
            rescuedVictims.Clear();
            RefreshVictimVisualizers();
        }
        // A Unity rescue is echoed back by ROS on the public count topic.
        // Do not treat that acknowledgement as another rescue command.
        int alreadyKnown = Math.Max(lastMissionRescuedCount, rescuedVictims.Count);
        int newRescues = Math.Max(0, rescuedCount - alreadyKnown);
        for (int index = 0; index < newRescues; index++)
        {
            SilenceNearestVictim();
        }
        lastMissionRescuedCount = rescuedCount;
    }

    void SilenceNearestVictim()
    {
        GameObject[] victims;
        try
        {
            victims = GameObject.FindGameObjectsWithTag(victimTag);
        }
        catch (UnityException)
        {
            return;
        }

        Transform nearest = null;
        float nearestDistance = float.PositiveInfinity;
        bool nearestIsTriggered = false;
        HashSet<int> visitedVictimRoots = new HashSet<int>();
        foreach (GameObject victim in victims)
        {
            Transform victimRoot = FindTaggedVictimRoot(victim.transform);
            int victimId = victimRoot.gameObject.GetInstanceID();
            if (!visitedVictimRoots.Add(victimId) || rescuedVictims.Contains(victimId))
                continue;

            float distance = PlanarDistance(sensingOrigin.position, victimRoot.position);
            VictimRangeVisualizer range =
                victimRoot.GetComponent<VictimRangeVisualizer>();
            bool triggered = range != null && range.IsRobotInside;
            if (triggered)
            {
                nearest = victimRoot;
                nearestDistance = distance;
                nearestIsTriggered = true;
                break;
            }
            if (distance < nearestDistance)
            {
                nearest = victimRoot;
                nearestDistance = distance;
            }
        }

        if (nearest != null)
        {
            if (!nearestIsTriggered && nearestDistance > rescueDistance)
            {
                Debug.LogWarning(
                    $"Rejected ROS rescue event: nearest victim is {nearestDistance:F2} m away " +
                    $"(limit {rescueDistance:F2} m).");
                return;
            }
            rescuedVictims.Add(nearest.gameObject.GetInstanceID());
            VictimRangeVisualizer visualizer =
                nearest.GetComponent<VictimRangeVisualizer>();
            if (visualizer != null) visualizer.SetRescued(true);
            Debug.Log($"Victim call silenced after ROS rescue event at {nearestDistance:F2} m.");
            RemoveRescuedVictim(nearest.gameObject);
        }
    }

    void RemoveRescuedVictim(GameObject victim)
    {
        if (victim == null) return;
        // Remove the complete Unity hierarchy after rescue while preserving
        // the episode denominator for the HUD and ROS total topic.
        Destroy(victim);
    }

    void RefreshVictimVisualizers()
    {
        GameObject[] victims;
        try
        {
            victims = GameObject.FindGameObjectsWithTag(victimTag);
        }
        catch (UnityException)
        {
            return;
        }

        HashSet<int> visited = new HashSet<int>();
        foreach (GameObject victim in victims)
        {
            Transform root = FindTaggedVictimRoot(victim.transform);
            int id = root.gameObject.GetInstanceID();
            if (!visited.Add(id)) continue;
            VictimRangeVisualizer visualizer =
                root.GetComponent<VictimRangeVisualizer>();
            if (visualizer == null)
                visualizer = root.gameObject.AddComponent<VictimRangeVisualizer>();
            visualizer.Configure(
                maximumHearingDistance,
                rescueDistance,
                rescuedVictims.Contains(id));
        }
    }

    static float PlanarDistance(Vector3 first, Vector3 second)
    {
        // Rescue/call ranges are drawn on Unity's XZ ground plane. Ignoring
        // height keeps the physical decision consistent with those rings and
        // with ROS, whose mission distance is computed in map X/Y.
        float dx = first.x - second.x;
        float dz = first.z - second.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    Transform FindTaggedVictimRoot(Transform candidate)
    {
        Transform root = candidate;
        while (root.parent != null && root.parent.CompareTag(victimTag))
        {
            root = root.parent;
        }
        return root;
    }

    static TimeMsg CurrentRosTime()
    {
        long milliseconds = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        return new TimeMsg(
            (uint)(milliseconds / 1000),
            (uint)((milliseconds % 1000) * 1_000_000));
    }
}

[DisallowMultipleComponent]
public class VictimRangeVisualizer : MonoBehaviour
{
    const int SegmentCount = 128;
    readonly Color callColor = new Color(1f, 0.82f, 0.05f, 0.72f);
    readonly Color rescueColor = new Color(1f, 0.25f, 0.05f, 0.95f);
    readonly Color completedColor = new Color(0.15f, 1f, 0.25f, 0.9f);

    LineRenderer callRing;
    LineRenderer rescueRing;
    VictimRescueTrigger rescueTrigger;
    VictimRescueTrigger callTrigger;
    float callRadius;
    float rescueRadius;
    bool rescued;

    public bool IsRobotInside =>
        rescueTrigger != null && rescueTrigger.IsRobotInside;

    public bool IsRobotInsideCall =>
        callTrigger != null && callTrigger.IsRobotInside;

    public void Configure(float callRange, float rescueRange, bool isRescued)
    {
        callRadius = Mathf.Max(0.1f, callRange);
        rescueRadius = Mathf.Max(0.1f, rescueRange);
        if (callRing == null) callRing = CreateRing("DistressCallRange", 0.10f);
        if (rescueRing == null) rescueRing = CreateRing("RescueDecisionRange", 0.16f);
        if (callTrigger == null)
        {
            GameObject triggerObject = new GameObject("DistressCallTrigger");
            triggerObject.layer = 0;
            triggerObject.transform.SetParent(transform, false);
            triggerObject.transform.localPosition = Vector3.up;
            callTrigger = triggerObject.AddComponent<VictimRescueTrigger>();
        }
        if (rescueTrigger == null)
        {
            GameObject triggerObject = new GameObject("RescueDecisionTrigger");
            triggerObject.layer = 0;
            triggerObject.transform.SetParent(transform, false);
            triggerObject.transform.localPosition = Vector3.up;
            rescueTrigger = triggerObject.AddComponent<VictimRescueTrigger>();
        }
        callTrigger.Configure(callRadius, !isRescued);
        rescueTrigger.Configure(rescueRadius, !isRescued);
        DrawRing(callRing, callRadius);
        DrawRing(rescueRing, rescueRadius);
        SetRescued(isRescued);
    }

    public void SetRescued(bool value)
    {
        rescued = value;
        if (callTrigger != null)
            callTrigger.Configure(callRadius, !rescued);
        if (rescueTrigger != null)
            rescueTrigger.Configure(rescueRadius, !rescued);
        SetRingColor(callRing, rescued ? completedColor : callColor);
        SetRingColor(rescueRing, rescued ? completedColor : rescueColor);
    }

    LineRenderer CreateRing(string ringName, float width)
    {
        GameObject ringObject = new GameObject(ringName);
        ringObject.transform.SetParent(transform, false);
        int ignoreLayer = LayerMask.NameToLayer("Ignore Raycast");
        if (ignoreLayer >= 0) ringObject.layer = ignoreLayer;

        LineRenderer line = ringObject.AddComponent<LineRenderer>();
        line.useWorldSpace = false;
        line.loop = true;
        line.positionCount = SegmentCount;
        line.startWidth = width;
        line.endWidth = width;
        line.numCornerVertices = 2;
        line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        line.receiveShadows = false;
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null) shader = Shader.Find("Unlit/Color");
        if (shader != null) line.material = new Material(shader);
        return line;
    }

    void DrawRing(LineRenderer line, float radius)
    {
        float localHeight = 0.04f;
        foreach (Collider collider in GetComponentsInChildren<Collider>())
        {
            if (collider.isTrigger) continue;
            localHeight = transform.InverseTransformPoint(collider.bounds.min).y + 0.04f;
            break;
        }
        for (int index = 0; index < SegmentCount; index++)
        {
            float angle = 2f * Mathf.PI * index / SegmentCount;
            line.SetPosition(index, new Vector3(
                Mathf.Cos(angle) * radius,
                localHeight,
                Mathf.Sin(angle) * radius));
        }
    }

    static void SetRingColor(LineRenderer line, Color color)
    {
        if (line == null) return;
        line.startColor = color;
        line.endColor = color;
        if (line.material != null) line.material.color = color;
    }
}

[DisallowMultipleComponent]
public class VictimRescueTrigger : MonoBehaviour
{
    readonly HashSet<int> robotColliderIds = new HashSet<int>();
    SphereCollider triggerCollider;

    public bool IsRobotInside =>
        triggerCollider != null && triggerCollider.enabled &&
        robotColliderIds.Count > 0;

    public void Configure(float radius, bool active)
    {
        if (triggerCollider == null)
        {
            triggerCollider = gameObject.GetComponent<SphereCollider>();
            if (triggerCollider == null)
                triggerCollider = gameObject.AddComponent<SphereCollider>();
            triggerCollider.isTrigger = true;
        }
        triggerCollider.radius = Mathf.Max(0.1f, radius);
        triggerCollider.enabled = active;
        if (!active) robotColliderIds.Clear();
    }

    void OnTriggerEnter(Collider other)
    {
        TrackRobot(other);
    }

    void OnTriggerStay(Collider other)
    {
        TrackRobot(other);
    }

    void OnTriggerExit(Collider other)
    {
        robotColliderIds.Remove(other.GetInstanceID());
    }

    void OnDisable()
    {
        robotColliderIds.Clear();
    }

    void TrackRobot(Collider other)
    {
        if (other.GetComponentInParent<CmdVelSubscriber>() != null)
            robotColliderIds.Add(other.GetInstanceID());
    }
}

public static class VictimCallSensorBootstrap
{
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void AttachToRobotRoot()
    {
        foreach (GameObject candidate in GameObject.FindGameObjectsWithTag("Robot"))
        {
            if (candidate.GetComponent<CmdVelSubscriber>() == null) continue;
            if (candidate.GetComponent<VictimCallSensor>() == null)
            {
                candidate.AddComponent<VictimCallSensor>();
            }
            return;
        }
        Debug.LogWarning("VictimCallSensor could not find the Robot root with CmdVelSubscriber.");
    }
}
