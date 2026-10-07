using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

[RequireComponent(typeof(Rigidbody))]
public class CmdVelSubscriber : MonoBehaviour
{
    public string topicName = "/cmd_vel";
    // Tolerate a short ROS-TCP scheduling gap without stopping the vehicle;
    // a much longer gap still fails safe.
    public float commandTimeout = 1.5f;
    public string motionBlockedTopic = "/unity_motion_guard/blocked";
    public string motionStatusTopic = "/unity_motion_guard/status";
    public string appliedCommandTopic = "/unity_motion_guard/applied_cmd";
    public float motionStatusRate = 5f;
    public float linearAcceleration = 2.5f;
    public float linearDeceleration = 6f;
    public float angularAcceleration = 3f;
    [Tooltip("Legacy CMU corner slowdown. Collision checks remain active when this slowdown is disabled.")]
    public bool enableLegacyCornerSlowdown = true;
    public float cornerSlowdownStart = 0.15f;
    public float cornerSlowdownFull = 0.70f;
    [Range(0f, 1f)] public float minimumCornerSpeedRatio = 0.18f;
    public float wallContactGraceTime = 0.35f;
    [Range(0f, 1f)] public float blockingDotThreshold = 0.1f;
    // ROS owns the recovery state machine. Unity only enforces predictive and
    // physical collision safety; running two independent recovery controllers
    // made their reverse/turn commands fight at wall corners.
    public bool enableContactRecovery = false;
    public float contactRecoveryDelay = 0.45f;
    public float contactRecoveryBackUpDuration = 1.6f;
    public float contactRecoveryTurnDuration = 0.75f;
    public float contactRecoveryCooldown = 0.8f;
    public float contactRecoveryReverseSpeed = 0.75f;
    public float contactRecoveryTurnRate = 0.9f;
    public float contactRecoveryTurnCommandThreshold = 0.45f;
    public float contactRecoveryStallSpeed = 0.15f;
    [Header("Generic physical stall detection")]
    [Tooltip("Report a blocked motion when commanded movement produces almost no physical translation or yaw while contact is present.")]
    public bool enablePhysicalStallSignal = true;
    [Min(0.2f)] public float physicalStallDelay = 1.25f;
    [Min(0f)] public float physicalStallLinearCommandThreshold = 0.12f;
    [Min(0f)] public float physicalStallAngularCommandThreshold = 0.40f;
    [Min(0f)] public float physicalStallSpeedThreshold = 0.06f;
    [Min(0f)] public float physicalStallYawRateThreshold = 0.12f;
    public bool enablePredictiveClearance = true;
    public string environmentLayerName = "Environment";
    public float predictiveLookAheadTime = 0.35f;
    [Range(1, 6)] public int predictivePoseSamples = 3;
    public float predictiveClearanceMargin = 0.35f;
    public float predictiveGroundIgnoreHeight = 0.2f;
    public float marginExitMaximumSpeed = 0.6f;
    public bool faceMovementDirection = true;
    public bool allowReverseMotion = true;
    public float movementAlignmentSpeed = 180f;
    public float movementAlignmentThreshold = 5f;
    public string collisionCountTopic = "/collision_count";
    public float collisionCountCooldown = 0.25f;
    [Header("Physics drive")]
    public bool reduceGroundFriction = true;
    [Range(0f, 1f)] public float groundStaticFriction = 0.05f;
    [Range(0f, 1f)] public float groundDynamicFriction = 0.03f;
    [Header("Visual rescue range")]
    public bool showRangeRing = true;
    [Min(0.1f)] public float rangeRingRadius = 3.0f;

    private ROSConnection ros;
    private Rigidbody body;
    private float targetLinearVelocity;
    private float targetAngularVelocity;
    private float appliedLinearVelocity;
    private float appliedAngularVelocity;
    private float lastCommandTime = float.NegativeInfinity;
    private uint commandReceiveCount;
    private Vector3 blockingWallNormal;
    private float lastBlockingContactTime = float.NegativeInfinity;
    private Vector3 wallEscapeNormal;
    private float wallContactStartTime = float.NegativeInfinity;
    private float lastWallContactTime = float.NegativeInfinity;
    private float recoveryStartTime = float.NegativeInfinity;
    private float recoveryEndTime = float.NegativeInfinity;
    private float recoveryTurnSign = 1f;
    private float recoveryTranslationSign = -1f;
    private BoxCollider chassisCollider;
    private Collider[] vehicleColliders;
    private int environmentLayerMask;
    private readonly Collider[] predictiveHits = new Collider[32];
    private readonly Collider[] marginExitObstacles = new Collider[32];
    private readonly float[] marginExitDistances = new float[32];
    private float lastMotionStatusTime = float.NegativeInfinity;
    private string lastMotionStatus = "";
    private Vector3 previousPhysicsPosition;
    private Vector3 previousAssignedVelocity;
    private float previousPhysicsYaw;
    private float previousAssignedYawRate;
    private bool havePhysicsSample;
    private float nextPhysicsDiagnostic;
    private string lastContactObject = "none";
    private float lastContactNormalY;
    private float lastContactDiagnosticTime = float.NegativeInfinity;
    private float physicalStallStartTime = float.NegativeInfinity;
    private bool predictiveQuerySaturated;
    public string MotionStatus => lastMotionStatus;
    private float lastCountedCollisionTime = float.NegativeInfinity;
    private uint collisionCount;

    void Start()
    {
        if (showRangeRing)
        {
            var ring = GetComponent<RobotRangeRing>();
            if (ring == null) ring = gameObject.AddComponent<RobotRangeRing>();
            ring.radius = rangeRingRadius;
            ring.visible = true;
        }
        body = GetComponent<Rigidbody>();
        chassisCollider = GetComponent<BoxCollider>();
        vehicleColliders = GetComponentsInChildren<Collider>();
        if (reduceGroundFriction)
        {
            // Direct velocity control should not be consumed by default high
            // friction on the chassis/wheel colliders.  Collision geometry is
            // retained; only tangential friction is reduced.
            var material = new PhysicsMaterial("RobotLowFriction")
            {
                staticFriction = groundStaticFriction,
                dynamicFriction = groundDynamicFriction,
                bounciness = 0f,
                frictionCombine = PhysicsMaterialCombine.Minimum,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };
            foreach (var collider in vehicleColliders)
            {
                if (collider != null && !collider.isTrigger) collider.sharedMaterial = material;
            }
        }
        int environmentLayer = LayerMask.NameToLayer(environmentLayerName);
        environmentLayerMask = environmentLayer >= 0 ? 1 << environmentLayer : 0;
        if (enablePredictiveClearance &&
            (chassisCollider == null || environmentLayerMask == 0))
        {
            Debug.LogWarning(
                "Predictive wall clearance disabled: a root BoxCollider and " +
                $"the '{environmentLayerName}' layer are required.",
                this);
            enablePredictiveClearance = false;
        }
        body.constraints |= RigidbodyConstraints.FreezeRotationX |
                            RigidbodyConstraints.FreezeRotationZ;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        if (!body.isKinematic)
        {
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        }
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<UInt32Msg>(collisionCountTopic, 1, true);
        ros.RegisterPublisher<BoolMsg>(motionBlockedTopic, 1);
        ros.RegisterPublisher<StringMsg>(motionStatusTopic, 1);
        ros.RegisterPublisher<TwistStampedMsg>(appliedCommandTopic, 1);
        ros.RegisterPublisher<StringMsg>("/unity_motion_guard/physics_diagnostic", 1);
        ros.Subscribe<TwistStampedMsg>(topicName, CmdVelCallback);
        // /cmd_vel is the sole motion authority. Mission turn requests are
        // arbitrated in ROS, before recovery and collision safety, not here.
        ros.Publish(collisionCountTopic, new UInt32Msg(0));
    }

    void CmdVelCallback(TwistStampedMsg msg)
    {
        commandReceiveCount++;
        float requestedLinearVelocity = (float)msg.twist.linear.x;
        if (float.IsNaN(requestedLinearVelocity) || float.IsInfinity(requestedLinearVelocity) ||
            double.IsNaN(msg.twist.angular.z) || double.IsInfinity(msg.twist.angular.z))
        {
            targetLinearVelocity = targetAngularVelocity = 0f;
            lastCommandTime = float.NegativeInfinity;
            return;
        }
        targetLinearVelocity = allowReverseMotion
            ? requestedLinearVelocity
            : Mathf.Max(0f, requestedLinearVelocity);
        targetAngularVelocity = (float)-msg.twist.angular.z;
        lastCommandTime = Time.time;
    }

    void FixedUpdate()
    {
        // Observe the previous physics step BEFORE writing the next velocity.
        // This is a measurement only; it does not alter contact or driving.
        Vector3 measuredStepVelocity = havePhysicsSample
            ? (body.position - previousPhysicsPosition) / Time.fixedDeltaTime
            : Vector3.zero;
        float measuredYawRate = havePhysicsSample
            ? Mathf.DeltaAngle(previousPhysicsYaw, body.rotation.eulerAngles.y) *
              Mathf.Deg2Rad / Mathf.Max(0.0001f, Time.fixedDeltaTime)
            : 0f;
        if (havePhysicsSample && Time.unscaledTime >= nextPhysicsDiagnostic)
        {
            nextPhysicsDiagnostic = Time.unscaledTime + 0.5f;
            Vector3 actual = body.linearVelocity;
            var commandTopicState = ros != null ? ros.GetTopic(topicName) : null;
            float transportCommandAge = commandTopicState != null &&
                                        commandTopicState.LastMessageReceivedRealtime > 0f
                ? Time.realtimeSinceStartup - commandTopicState.LastMessageReceivedRealtime
                : float.PositiveInfinity;
            string diagnostic = System.FormattableString.Invariant(
                $"previous_assigned_world=({previousAssignedVelocity.x:F4},{previousAssignedVelocity.z:F4}) current_body_world=({actual.x:F4},{actual.z:F4}) displacement_world=({measuredStepVelocity.x:F4},{measuredStepVelocity.z:F4}) yaw_ros_assigned={-previousAssignedYawRate:F4} yaw_ros_body={-body.angularVelocity.y:F4} yaw_ros_displacement={-measuredYawRate:F4} command_transport_age={transportCommandAge:F3} command_callback_age={Time.time-lastCommandTime:F3} command_callbacks={commandReceiveCount} command_timeout={commandTimeout:F3} last_contact={lastContactObject} normal_y={lastContactNormalY:F3} contact_age={Time.time-lastContactDiagnosticTime:F3}");
            ros.Publish("/unity_motion_guard/physics_diagnostic", new StringMsg(diagnostic));
        }
        previousPhysicsPosition = body.position;
        previousPhysicsYaw = body.rotation.eulerAngles.y;
        havePhysicsSample = true;
        bool commandStale = Time.time - lastCommandTime > commandTimeout;
        if (commandStale)
        {
            targetLinearVelocity = 0f;
            targetAngularVelocity = 0f;
        }

        float requestedLinearVelocity = targetLinearVelocity;
        float requestedAngularVelocity = targetAngularVelocity;

        bool wallContactActive = Time.time - lastWallContactTime <= wallContactGraceTime;
        if (!wallContactActive)
        {
            wallContactStartTime = float.NegativeInfinity;
        }

        Vector3 planarBodyVelocity = Vector3.ProjectOnPlane(body.linearVelocity, Vector3.up);
        bool recoveryActive = Time.time < recoveryEndTime;
        bool commandedAgainstCorner =
            Mathf.Abs(requestedAngularVelocity) >= contactRecoveryTurnCommandThreshold ||
            Mathf.Abs(requestedLinearVelocity) > 0.1f;
        bool contactStalled =
            wallContactActive &&
            commandedAgainstCorner &&
            planarBodyVelocity.magnitude <= contactRecoveryStallSpeed;

        // Generic physical-stall evidence. This deliberately does not inspect
        // collider names or shapes: a wall, tank, crate, or arbitrary mesh is
        // treated identically when a meaningful command produces neither
        // translation nor yaw while physical contact remains fresh.
        bool meaningfulMotionCommand =
            Mathf.Abs(requestedLinearVelocity) >= physicalStallLinearCommandThreshold ||
            Mathf.Abs(requestedAngularVelocity) >= physicalStallAngularCommandThreshold;
        bool noMeasuredProgress =
            planarBodyVelocity.magnitude <= physicalStallSpeedThreshold &&
            Mathf.Abs(measuredYawRate) <= physicalStallYawRateThreshold;
        bool physicalStallEvidence = enablePhysicalStallSignal &&
            wallContactActive && meaningfulMotionCommand && noMeasuredProgress;
        if (physicalStallEvidence)
        {
            if (float.IsNegativeInfinity(physicalStallStartTime))
                physicalStallStartTime = Time.time;
        }
        else
        {
            physicalStallStartTime = float.NegativeInfinity;
        }
        bool physicalStallBlocked = physicalStallEvidence &&
            Time.time - physicalStallStartTime >= physicalStallDelay;

        if (enableContactRecovery &&
            !recoveryActive &&
            Time.time >= recoveryEndTime + contactRecoveryCooldown &&
            contactStalled &&
            Time.time - wallContactStartTime >= contactRecoveryDelay)
        {
            // Contact normals point from the wall toward the robot. Their
            // horizontal sum therefore points toward the open side of an
            // inside corner. Turn the nose in that direction while reversing
            // far enough to restore a usable turning envelope.
            float side = Vector3.Dot(wallEscapeNormal, transform.right);
            if (Mathf.Abs(side) < 0.1f)
            {
                side = -Mathf.Sign(requestedAngularVelocity);
                if (Mathf.Abs(side) < 0.1f) side = 1f;
            }

            recoveryTurnSign = Mathf.Sign(side);
            float longitudinalEscape =
                Vector3.Dot(wallEscapeNormal, transform.forward);
            if (Mathf.Abs(longitudinalEscape) >= 0.12f)
            {
                // A fixed reverse command can push the robot deeper into a
                // corner when its rear is the blocked end. Select whichever
                // longitudinal direction actually points toward free space.
                recoveryTranslationSign = Mathf.Sign(longitudinalEscape);
            }
            else
            {
                // For a nearly pure side contact, retreat opposite the most
                // recent forward command before attempting the turn.
                recoveryTranslationSign = requestedLinearVelocity > 0.05f
                    ? -1f
                    : -1f;
            }
            recoveryStartTime = Time.time;
            recoveryEndTime = Time.time +
                Mathf.Max(0f, contactRecoveryBackUpDuration) +
                Mathf.Max(0f, contactRecoveryTurnDuration);
            // Cancel any angular momentum inherited from the failed command.
            // The first recovery phase must be a genuinely straight retreat.
            appliedLinearVelocity = 0f;
            appliedAngularVelocity = 0f;
            recoveryActive = true;
        }

        if (recoveryActive)
        {
            float recoveryElapsed = Time.time - recoveryStartTime;
            if (recoveryElapsed < contactRecoveryBackUpDuration)
            {
                // Phase 1: create clearance without sweeping a vehicle corner
                // farther into the wall.
                requestedLinearVelocity =
                    recoveryTranslationSign * Mathf.Abs(contactRecoveryReverseSpeed);
                requestedAngularVelocity = 0f;
            }
            else
            {
                // Phase 2: only after backing clear, rotate toward the open
                // side. Translation remains zero until the planner resumes.
                requestedLinearVelocity = 0f;
                requestedAngularVelocity =
                    recoveryTurnSign * Mathf.Abs(contactRecoveryTurnRate);
            }
        }

        // CMU may request full cruise speed while beginning a sharp corner.
        // Slow the tracked robot before the turn so its physical collider
        // follows the planned centreline instead of clipping the inside wall.
        if (!recoveryActive && enableLegacyCornerSlowdown)
        {
            float turnAmount = Mathf.InverseLerp(
                cornerSlowdownStart,
                Mathf.Max(cornerSlowdownStart + 0.01f, cornerSlowdownFull),
                Mathf.Abs(requestedAngularVelocity));
            requestedLinearVelocity *= Mathf.Lerp(1f, minimumCornerSpeedRatio, turnAmount);
        }

        float linearRate = Mathf.Abs(requestedLinearVelocity) < Mathf.Abs(appliedLinearVelocity)
            ? linearDeceleration
            : linearAcceleration;
        appliedLinearVelocity = Mathf.MoveTowards(
            appliedLinearVelocity,
            requestedLinearVelocity,
            linearRate * Time.fixedDeltaTime);
        appliedAngularVelocity = Mathf.MoveTowards(
            appliedAngularVelocity,
            requestedAngularVelocity,
            angularAcceleration * Time.fixedDeltaTime);

        // Check the motion that will actually be applied, after acceleration.
        // A safety veto must stop immediately, not coast through the margin.
        bool safetyBlocked = false;
        string motionStatus = commandStale ? "command_timeout" : "normal";
        if (commandStale)
        {
            appliedLinearVelocity = 0f;
            appliedAngularVelocity = 0f;
        }
        else if (enablePredictiveClearance && PredictFutureCollision(
                     appliedLinearVelocity, appliedAngularVelocity,
                     out Vector3 predictedEscapeNormal))
        {
            bool predictionWasInactive =
                Time.time - lastWallContactTime > wallContactGraceTime;
            wallEscapeNormal = predictedEscapeNormal;
            lastWallContactTime = Time.time;
            if (predictionWasInactive) wallContactStartTime = Time.time;
            float exitVelocity = Mathf.Clamp(appliedLinearVelocity,
                -Mathf.Abs(marginExitMaximumSpeed), Mathf.Abs(marginExitMaximumSpeed));
            if (TryFindSafeReducedArc(
                    appliedLinearVelocity,
                    appliedAngularVelocity,
                    out float safeLinearVelocity))
            {
                // Do not turn a conservative look-ahead hit into a permanent
                // zero-command deadlock.  Preserve the planner's chosen turn
                // and reduce only translation until the complete sampled arc
                // clears the inflated footprint.
                appliedLinearVelocity = safeLinearVelocity;
                motionStatus = "predictive_clearance_slowdown";
            }
            else if (Mathf.Abs(requestedAngularVelocity) < 0.001f &&
                Mathf.Abs(appliedAngularVelocity) < 0.001f &&
                CanExitClearanceMargin(exitVelocity))
            {
                // Not a new recovery direction: only permit the straight ROS
                // command if it monotonically leaves an existing safety margin.
                appliedLinearVelocity = exitVelocity;
                motionStatus = "clearance_margin_exit";
            }
            else if (Mathf.Abs(appliedLinearVelocity) < 0.001f &&
                     Mathf.Abs(appliedAngularVelocity) >= 0.001f &&
                     CanSafelyRotateInsideClearanceMargin(appliedAngularVelocity))
            {
                // The inflated safety box may overlap a nearby wall even
                // though the physical chassis still has room to pivot away.
                // Permit only a planner-requested pure rotation whose sampled
                // physical chassis poses remain penetration-free.  This is a
                // safety-filter exception, not a recovery direction choice.
                appliedLinearVelocity = 0f;
                motionStatus = "clearance_margin_safe_rotation";
            }
            else if (Mathf.Abs(appliedAngularVelocity) >= 0.001f &&
                     CanSafelyRotateInsideClearanceMargin(appliedAngularVelocity))
            {
                // A combined forward/turn command can sweep the front corner
                // into the inflated margin even when rotating in the same
                // direction is physically safe.  Rotate first so the next
                // planner command can leave the wall instead of repeatedly
                // receiving an all-zero veto.
                appliedLinearVelocity = 0f;
                motionStatus = "predictive_clearance_rotate_first";
            }
            else
            {
                appliedLinearVelocity = 0f;
                appliedAngularVelocity = 0f;
                safetyBlocked = true;
                motionStatus = "predictive_clearance_stop";
            }
        }

        Vector3 velocity = transform.forward * appliedLinearVelocity;
        bool velocityWasProjected = false;
        if (Time.time - lastBlockingContactTime <= wallContactGraceTime &&
            velocity.sqrMagnitude > 0.000001f &&
            Vector3.Dot(velocity.normalized, blockingWallNormal) < -blockingDotThreshold)
        {
            // Keep tangential motion but never re-apply velocity into a wall.
            velocity = Vector3.ProjectOnPlane(velocity, blockingWallNormal);
            velocityWasProjected = true;
        }

        body.linearVelocity = new Vector3(velocity.x, body.linearVelocity.y, velocity.z);
        previousAssignedVelocity = velocity;

        // Candidate orientation for alignment and safety checks. Dynamic
        // bodies execute the resulting yaw RATE, not an additional pose move.
        float nextYaw = body.rotation.eulerAngles.y +
                        appliedAngularVelocity * Mathf.Rad2Deg * Time.fixedDeltaTime;
        Quaternion nextRotation = Quaternion.Euler(0f, nextYaw, 0f);

        // Wall projection can turn a forward command into sideways motion.
        // Rotate the chassis toward that actual motion so all child sensors
        // continue looking in the direction in which the robot is travelling.
        if (!safetyBlocked && faceMovementDirection &&
            velocityWasProjected &&
            velocity.sqrMagnitude > 0.0001f)
        {
            Quaternion movementRotation = Quaternion.LookRotation(
                velocity.normalized * (appliedLinearVelocity < 0f ? -1f : 1f),
                Vector3.up);
            float alignmentAngle = Quaternion.Angle(body.rotation, movementRotation);
            if (alignmentAngle >= movementAlignmentThreshold)
            {
                nextRotation = Quaternion.RotateTowards(
                    body.rotation,
                    movementRotation,
                    movementAlignmentSpeed * Time.fixedDeltaTime);
            }
        }

        // Tangential alignment is also a rotation and must pass the same
        // footprint check. In reverse it must not flip the nose by 180 degrees.
        float effectiveAngularVelocity = Mathf.DeltaAngle(
            body.rotation.eulerAngles.y, nextRotation.eulerAngles.y) * Mathf.Deg2Rad /
            Mathf.Max(0.0001f, Time.fixedDeltaTime);
        if (velocityWasProjected && enablePredictiveClearance &&
            PredictFutureCollision(appliedLinearVelocity, effectiveAngularVelocity, out _))
        {
            nextRotation = body.rotation;
            effectiveAngularVelocity = 0f;
            appliedAngularVelocity = 0f;
        }
        PlanarDriveExecution.ApplyYaw(body, nextRotation, effectiveAngularVelocity);
        previousAssignedYawRate = effectiveAngularVelocity;
        bool reportedBlocked = safetyBlocked || physicalStallBlocked;
        if (physicalStallBlocked && !safetyBlocked) motionStatus = "physical_stall";
        PublishMotionStatus(reportedBlocked, motionStatus, velocity, effectiveAngularVelocity);
    }

    void PublishMotionStatus(bool blocked, string status, Vector3 velocity, float angularVelocity)
    {
        if (status == lastMotionStatus &&
            Time.time - lastMotionStatusTime < 1f / Mathf.Max(1f, motionStatusRate)) return;
        lastMotionStatus = status;
        lastMotionStatusTime = Time.time;
        ros.Publish(motionBlockedTopic, new BoolMsg(blocked));
        ros.Publish(motionStatusTopic, new StringMsg(status));
        var message = new TwistStampedMsg();
        var stamp = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        message.header.frame_id = "vehicle";
        message.header.stamp.sec = (uint)(stamp / 1000);
        message.header.stamp.nanosec = (uint)((stamp % 1000) * 1000000);
        message.twist.linear.x = Vector3.Dot(velocity, transform.forward);
        message.twist.angular.z = -angularVelocity;
        ros.Publish(appliedCommandTopic, message);
    }

    bool CanExitClearanceMargin(float linearVelocity)
    {
        if (Mathf.Abs(linearVelocity) < 0.001f) return false;
        if (!CheckEnvironmentOverlap(body.position, body.rotation, out _) ||
            predictiveQuerySaturated) return false;
        // A saturated overlap query cannot prove a safe exit.
        int obstacleCount = 0;
        Vector3 center = chassisCollider.bounds.center;
        foreach (Collider obstacle in predictiveHits)
        {
            if (!IsWallObstacle(obstacle, chassisCollider.bounds.min.y)) continue;
            if (obstacleCount >= marginExitObstacles.Length) return false;
            marginExitObstacles[obstacleCount] = obstacle;
            marginExitDistances[obstacleCount] =
                (center - obstacle.ClosestPoint(center)).sqrMagnitude;
            obstacleCount++;
        }
        if (obstacleCount == 0 || obstacleCount >= predictiveHits.Length) return false;

        int samples = Mathf.Max(3, predictivePoseSamples);
        float horizon = Mathf.Max(Time.fixedDeltaTime, predictiveLookAheadTime);
        for (int sample = 1; sample <= samples; sample++)
        {
            Vector3 shift = transform.forward * linearVelocity * horizon * sample / samples;
            Vector3 shiftedCenter = center + shift;
            bool improved = false;
            for (int index = 0; index < obstacleCount; index++)
            {
                Collider obstacle = marginExitObstacles[index];
                // Never permit physical penetration, even for an exit command.
                foreach (Collider part in vehicleColliders)
                {
                    if (part == null || !part.enabled || part.isTrigger ||
                        part.attachedRigidbody != body) continue;
                    if (Physics.ComputePenetration(part,
                            part.transform.position + shift, part.transform.rotation,
                            obstacle, obstacle.transform.position, obstacle.transform.rotation,
                            out _, out float penetration) && penetration > 0.0001f) return false;
                }
                float distance = (shiftedCenter - obstacle.ClosestPoint(shiftedCenter)).sqrMagnitude;
                if (distance < marginExitDistances[index] - 0.000001f) return false;
                improved |= distance > marginExitDistances[index] + 0.000001f;
                marginExitDistances[index] = distance;
            }
            if (!improved) return false;
            CheckEnvironmentOverlap(body.position + shift, body.rotation, out _);
            if (predictiveQuerySaturated) return false;
            foreach (Collider hit in predictiveHits)
            {
                if (!IsWallObstacle(hit, chassisCollider.bounds.min.y)) continue;
                bool existed = false;
                for (int index = 0; index < obstacleCount; index++)
                    existed |= hit == marginExitObstacles[index];
                if (!existed) return false;
            }
        }
        return true;
    }

    bool IsWallObstacle(Collider obstacle, float robotBottom)
    {
        return obstacle != null && !obstacle.transform.IsChildOf(transform) &&
            obstacle.bounds.max.y > robotBottom + predictiveGroundIgnoreHeight;
    }

    bool PredictFutureCollision(
        float linearVelocity,
        float angularVelocity,
        out Vector3 escapeNormal)
    {
        escapeNormal = Vector3.zero;
        if (chassisCollider == null || environmentLayerMask == 0) return false;
        if (Mathf.Abs(linearVelocity) < 0.001f &&
            Mathf.Abs(angularVelocity) < 0.001f) return false;

        int sampleCount = Mathf.Clamp(predictivePoseSamples, 1, 6);
        float horizon = Mathf.Max(Time.fixedDeltaTime, predictiveLookAheadTime);
        Vector3 accumulatedNormal = Vector3.zero;
        bool blocked = false;

        for (int i = 1; i <= sampleCount; i++)
        {
            float predictionTime = horizon * i / sampleCount;
            float yawDegrees = angularVelocity * Mathf.Rad2Deg * predictionTime;
            Quaternion predictedRotation =
                body.rotation * Quaternion.Euler(0f, yawDegrees, 0f);

            // Use the half-step heading to approximate the curved motion made
            // while translating and rotating at the same time.
            Quaternion travelRotation =
                body.rotation * Quaternion.Euler(0f, yawDegrees * 0.5f, 0f);
            Vector3 predictedPosition = body.position +
                travelRotation * Vector3.forward * linearVelocity * predictionTime;

            if (CheckEnvironmentOverlap(
                predictedPosition,
                predictedRotation,
                out Vector3 sampleNormal))
            {
                blocked = true;
                accumulatedNormal += sampleNormal;
            }
        }

        if (blocked)
        {
            escapeNormal = accumulatedNormal.sqrMagnitude > 0.01f
                ? accumulatedNormal.normalized
                : -transform.forward;
        }
        return blocked;
    }

    bool CanSafelyRotateInsideClearanceMargin(float angularVelocity)
    {
        if (chassisCollider == null || Mathf.Abs(angularVelocity) < 0.001f)
            return false;

        int samples = Mathf.Max(3, predictivePoseSamples);
        float horizon = Mathf.Max(Time.fixedDeltaTime, predictiveLookAheadTime);
        for (int sample = 1; sample <= samples; sample++)
        {
            float predictionTime = horizon * sample / samples;
            Quaternion predictedRotation = body.rotation * Quaternion.Euler(
                0f, angularVelocity * Mathf.Rad2Deg * predictionTime, 0f);

            CheckEnvironmentOverlap(body.position, predictedRotation, out _);
            if (predictiveQuerySaturated) return false;
            foreach (Collider obstacle in predictiveHits)
            {
                if (!IsWallObstacle(obstacle, chassisCollider.bounds.min.y))
                    continue;
                if (Physics.ComputePenetration(
                        chassisCollider,
                        body.position,
                        predictedRotation,
                        obstacle,
                        obstacle.transform.position,
                        obstacle.transform.rotation,
                        out _,
                        out float penetration) && penetration > 0.0001f)
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool TryFindSafeReducedArc(
        float linearVelocity,
        float angularVelocity,
        out float safeLinearVelocity)
    {
        safeLinearVelocity = 0f;
        if (Mathf.Abs(linearVelocity) < 0.05f ||
            Mathf.Abs(angularVelocity) < 0.001f) return false;

        // Keep angular authority and search from the fastest useful arc down.
        // Every candidate is checked by the same inflated-footprint predictor,
        // so this relaxes liveness without relaxing collision clearance.
        float[] speedRatios = { 0.70f, 0.45f, 0.25f, 0.12f };
        foreach (float ratio in speedRatios)
        {
            float candidate = linearVelocity * ratio;
            if (!PredictFutureCollision(candidate, angularVelocity, out _))
            {
                safeLinearVelocity = candidate;
                return true;
            }
        }
        return false;
    }

    bool CheckEnvironmentOverlap(
        Vector3 rootPosition,
        Quaternion rootRotation,
        out Vector3 escapeNormal)
    {
        escapeNormal = Vector3.zero;
        Vector3 scale = transform.lossyScale;
        scale = new Vector3(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z));
        Vector3 scaledCenter = Vector3.Scale(chassisCollider.center, scale);
        Vector3 halfExtents = Vector3.Scale(chassisCollider.size, scale) * 0.5f;
        halfExtents.x += Mathf.Max(0f, predictiveClearanceMargin);
        halfExtents.z += Mathf.Max(0f, predictiveClearanceMargin);

        Vector3 boxCenter = rootPosition + rootRotation * scaledCenter;
        int hitCount = Physics.OverlapBoxNonAlloc(
            boxCenter,
            halfExtents,
            predictiveHits,
            rootRotation,
            environmentLayerMask,
            QueryTriggerInteraction.Ignore);

        // NonAlloc leaves old entries beyond hitCount; clear them before any
        // margin-exit proof reads this buffer.
        for (int index = hitCount; index < predictiveHits.Length; index++)
            predictiveHits[index] = null;
        predictiveQuerySaturated = hitCount >= predictiveHits.Length;
        if (predictiveQuerySaturated)
        {
            escapeNormal = -transform.forward;
            return true;
        }

        if (hitCount == 0) return false;

        float robotBottom = boxCenter.y - halfExtents.y;
        Vector3 normalSum = Vector3.zero;
        bool foundWall = false;
        for (int i = 0; i < hitCount; i++)
        {
            Collider obstacle = predictiveHits[i];
            if (obstacle == null || obstacle.transform.IsChildOf(transform)) continue;

            // The floor shares the Environment layer. Ignore surfaces whose
            // top stays close to the robot footprint bottom, while retaining
            // walls, crates and other vertically substantial obstacles.
            if (obstacle.bounds.max.y <=
                robotBottom + predictiveGroundIgnoreHeight) continue;

            foundWall = true;
            Vector3 closest = obstacle.ClosestPoint(boxCenter);
            Vector3 horizontalNormal =
                Vector3.ProjectOnPlane(boxCenter - closest, Vector3.up);
            if (horizontalNormal.sqrMagnitude < 0.01f)
            {
                horizontalNormal = Vector3.ProjectOnPlane(
                    boxCenter - obstacle.bounds.center,
                    Vector3.up);
            }
            if (horizontalNormal.sqrMagnitude > 0.01f)
            {
                normalSum += horizontalNormal.normalized;
            }
        }

        if (foundWall)
        {
            escapeNormal = normalSum.sqrMagnitude > 0.01f
                ? normalSum.normalized
                : -transform.forward;
        }
        return foundWall;
    }

    void OnCollisionStay(Collision collision)
    {
        lastContactObject = collision.gameObject.name;
        lastContactDiagnosticTime = Time.time;
        lastContactNormalY = collision.contactCount > 0 ? collision.GetContact(0).normal.y : 0f;
        Vector3 commandDirection =
            Vector3.ProjectOnPlane(body.linearVelocity, Vector3.up);
        if (commandDirection.sqrMagnitude > 0.000001f)
        {
            commandDirection.Normalize();
        }
        else
        {
            commandDirection =
                transform.forward * Mathf.Sign(targetLinearVelocity);
        }
        Vector3 allWallNormals = Vector3.zero;
        Vector3 blockingNormals = Vector3.zero;

        foreach (ContactPoint contact in collision.contacts)
        {
            Vector3 horizontalNormal = Vector3.ProjectOnPlane(contact.normal, Vector3.up);
            if (horizontalNormal.sqrMagnitude < 0.01f) continue;

            horizontalNormal.Normalize();
            allWallNormals += horizontalNormal;
            if (commandDirection.sqrMagnitude > 0.000001f &&
                Vector3.Dot(commandDirection, horizontalNormal) < -blockingDotThreshold)
            {
                blockingNormals += horizontalNormal;
            }
        }

        if (allWallNormals.sqrMagnitude > 0.01f)
        {
            bool contactWasInactive = Time.time - lastWallContactTime > wallContactGraceTime;
            wallEscapeNormal = allWallNormals.normalized;
            lastWallContactTime = Time.time;
            if (contactWasInactive) wallContactStartTime = Time.time;
        }

        if (blockingNormals.sqrMagnitude > 0.01f)
        {
            blockingWallNormal = blockingNormals.normalized;
            lastBlockingContactTime = Time.time;
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        if (Time.time - lastCountedCollisionTime < collisionCountCooldown) return;

        foreach (ContactPoint contact in collision.contacts)
        {
            // Ignore the floor and count contacts with a substantial
            // horizontal normal as wall/object collisions.
            Vector3 horizontalNormal = Vector3.ProjectOnPlane(contact.normal, Vector3.up);
            if (horizontalNormal.sqrMagnitude < 0.25f) continue;

            collisionCount++;
            lastCountedCollisionTime = Time.time;
            ros.Publish(collisionCountTopic, new UInt32Msg(collisionCount));
            break;
        }
    }
}
