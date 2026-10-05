using System.Collections.Generic;
using UnityEngine;

public class LidarSensor : MonoBehaviour
{
    [Header("Detection Settings")]
    [Tooltip("Layers that the LiDAR rays are allowed to detect.")]
    public LayerMask detectionMask = ~0;

    [Header("Scan Settings")]
    // Low-load profile: 1,260 rays/scan. At 6 Hz this is about 61% lighter
    // than the former 20 x 120 x 8 Hz profile while preserving 360 degrees.
    public int verticalScanLines = 14;
    public int horizontalScanLines = 90;
    public float maxDistance = 2000f;
    public float scanRateHz = 6f;

    [Header("Vertical Field Of View (degrees)")]
    public float verticalFovLower = -15f;
    public float verticalFovUpper = 15f;

    [Header("Debug Visualization")]
    public bool drawDebugRays = false;

    public List<Vector3> pointCloudData = new List<Vector3>();

    // Colliders that have really been observed by LiDAR.  Runtime UI uses
    // this collection to distinguish discovered structures from objects that
    // merely exist in the Unity scene.
    public IReadOnlyCollection<Collider> DetectedEnvironmentColliders => detectedEnvironmentColliders;

    private float timer;
    private readonly HashSet<Collider> detectedEnvironmentColliders = new HashSet<Collider>();

    void Update()
    {
        if (scanRateHz <= 0f) return;

        timer += Time.deltaTime;
        float interval = 1f / scanRateHz;
        if (timer < interval) return;

        timer %= interval;
        PerformLidarScan();
    }

    void PerformLidarScan()
    {
        pointCloudData.Clear();

        int verticalLines = Mathf.Max(1, verticalScanLines);
        int horizontalLines = Mathf.Max(1, horizontalScanLines);
        float angleStepH = 360f / horizontalLines;
        float angleStepV = verticalLines == 1
            ? 0f
            : (verticalFovUpper - verticalFovLower) / (verticalLines - 1);

        for (int v = 0; v < verticalLines; v++)
        {
            float verticalAngle = verticalFovLower + v * angleStepV;

            for (int h = 0; h < horizontalLines; h++)
            {
                float horizontalAngle = h * angleStepH;
                Quaternion rayRotation = Quaternion.Euler(verticalAngle, horizontalAngle, 0f);
                Vector3 rayDirection = transform.rotation * rayRotation * Vector3.forward;

                if (Physics.Raycast(
                    transform.position,
                    rayDirection,
                    out RaycastHit hit,
                    maxDistance,
                    detectionMask,
                    QueryTriggerInteraction.Ignore))
                {
                    pointCloudData.Add(hit.point);
                    RememberDetectedEnvironment(hit.collider);

                    if (drawDebugRays)
                    {
                        Debug.DrawLine(transform.position, hit.point, Color.cyan, 1f / scanRateHz);
                    }
                }
                else if (drawDebugRays)
                {
                    Debug.DrawRay(
                        transform.position,
                        rayDirection * maxDistance,
                        new Color(1f, 0f, 0f, 0.1f),
                        1f / scanRateHz);
                }
            }
        }
    }

    void RememberDetectedEnvironment(Collider hitCollider)
    {
        if (hitCollider == null) return;

        int environmentLayer = LayerMask.NameToLayer("Environment");
        if (environmentLayer >= 0 && hitCollider.gameObject.layer == environmentLayer)
        {
            detectedEnvironmentColliders.Add(hitCollider);
        }
    }

    void OnDrawGizmosSelected()
    {
        if (pointCloudData == null) return;

        Gizmos.color = Color.red;
        foreach (Vector3 point in pointCloudData)
        {
            Gizmos.DrawSphere(point, 0.05f);
        }
    }
}
