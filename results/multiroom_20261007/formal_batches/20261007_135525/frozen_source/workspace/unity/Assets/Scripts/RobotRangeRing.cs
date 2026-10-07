using UnityEngine;

/// Visual-only configurable circular range around the robot.
[RequireComponent(typeof(LineRenderer))]
public sealed class RobotRangeRing : MonoBehaviour
{
    [Min(0.1f)] public float radius = 3f;
    [Range(16, 256)] public int segments = 96;
    [Min(0.001f)] public float lineWidth = 0.035f;
    public Color color = new Color(0.1f, 1f, 0.2f, 0.75f);
    public bool visible = true;
    public float height = 0.03f;
    LineRenderer line;

    void Awake() { line = GetComponent<LineRenderer>(); Configure(); Rebuild(); }
    void OnValidate() { if (line == null) line = GetComponent<LineRenderer>(); if (line != null) { Configure(); Rebuild(); } }
    void Update() { if (line == null) return; line.enabled = visible; line.startColor = line.endColor = color; line.startWidth = line.endWidth = lineWidth; }
    void Configure()
    {
        line.useWorldSpace = false; line.loop = true; line.alignment = LineAlignment.TransformZ;
        line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off; line.receiveShadows = false;
        if (line.sharedMaterial == null) line.material = new Material(Shader.Find("Sprites/Default"));
    }
    void Rebuild()
    {
        if (line == null) return; segments = Mathf.Clamp(segments, 16, 256); radius = Mathf.Max(.1f, radius);
        line.positionCount = segments;
        for (int i = 0; i < segments; i++) { float a = 2f * Mathf.PI * i / segments; line.SetPosition(i, new Vector3(Mathf.Cos(a) * radius, height, Mathf.Sin(a) * radius)); }
        line.startWidth = line.endWidth = lineWidth; line.startColor = line.endColor = color; line.enabled = visible;
    }
}
