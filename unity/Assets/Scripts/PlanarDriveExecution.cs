using UnityEngine;

// Final actuator step, called only AFTER acceleration and collision checks.
// Keep dynamic-body rotation under one authority: angular velocity, including
// zero. Combining a pose move with inherited angular velocity causes drift.
public static class PlanarDriveExecution
{
    public static void ApplyYaw(Rigidbody body, Quaternion targetRotation, float yawRate)
    {
        if (body.isKinematic)
            body.MoveRotation(targetRotation);
        else
            body.angularVelocity = Vector3.up * yawRate;
    }
}
