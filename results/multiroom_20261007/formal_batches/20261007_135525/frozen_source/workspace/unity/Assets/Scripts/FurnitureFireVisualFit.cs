using UnityEngine;

// Presentation sizing only. Does not change colliders, fire truth or semantic masks.
public static class FurnitureFireVisualFit
{
    public static void Apply(GameObject furniture, GameObject fire, float coverage, float intensity)
    {
        var meshes=furniture.GetComponentsInChildren<MeshRenderer>();
        if(meshes.Length==0)return;
        Bounds bounds=meshes[0].bounds;
        foreach(var mesh in meshes)if(mesh.enabled)bounds.Encapsulate(mesh.bounds);
        float width=Mathf.Max(.5f,bounds.size.x*coverage);
        float depth=Mathf.Max(.5f,bounds.size.z*coverage);
        // Low broad furniture burns near its upper surface; tall furniture burns
        // through its upper interior, leaving clearance below the 5-unit ceiling.
        float baseY=bounds.min.y+Mathf.Min(bounds.size.y*.65f,1.7f);
        float height=Mathf.Clamp(Mathf.Max(width,depth)*.5f,1.2f,2.6f);
        height=Mathf.Min(height,Mathf.Max(.5f,4.7f-baseY));
        foreach(var ps in fire.GetComponentsInChildren<ParticleSystem>(true))
        {
            ps.Stop(true,ParticleSystemStopBehavior.StopEmittingAndClear);
            ps.transform.position=new Vector3(bounds.center.x,baseY,bounds.center.z);
            ps.transform.rotation=Quaternion.Euler(-90,0,0);
            Vector3 p=ps.transform.parent.lossyScale;
            ps.transform.localScale=new Vector3(1/Mathf.Max(.001f,Mathf.Abs(p.x)),1/Mathf.Max(.001f,Mathf.Abs(p.y)),1/Mathf.Max(.001f,Mathf.Abs(p.z)));
            var shape=ps.shape;shape.shapeType=ParticleSystemShapeType.Box;
            shape.scale=new Vector3(width,depth,.15f);
            var main=ps.main;main.scalingMode=ParticleSystemScalingMode.Hierarchy;
            main.startLifetime=1.1f;main.startSpeed=new ParticleSystem.MinMaxCurve(height*.65f,height*.9f);
            bool smoke=ps.name=="Smoke", ember=ps.name=="Embers";
            float size=ember?.14f:Mathf.Clamp(Mathf.Min(width,depth)*.75f,.85f,2f);
            main.startSize=new ParticleSystem.MinMaxCurve(size*.65f,size);
            main.maxParticles=700;
            var emission=ps.emission;
            emission.rateOverTime=Mathf.Clamp(width*depth*(smoke?4:ember?3:22)*intensity,smoke?12:ember?10:55,smoke?70:ember?45:280);
        }
    }
}
