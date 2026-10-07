using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

// Visual surrogate only: no heat transfer, combustion or smoke attenuation model.
[InitializeOnLoad]
public static class RescueFireEffects
{
    static RescueFireEffects(){EditorApplication.delayCall+=AutoInstall;}
    static void AutoInstall()
    {
        var scene=SceneManager.GetActiveScene();
        if(!EditorApplication.isPlayingOrWillChangePlaymode && !scene.isDirty &&
           scene.path==FurnishedRescueScene.PathName && GameObject.Find("FireVisualEffects")==null)Install();
    }
    static Material ParticleMaterial()
    {
        const string path="Assets/Msterial/RescueFireParticle.mat";
        var mat=AssetDatabase.LoadAssetAtPath<Material>(path);
        if(mat!=null)return mat;
        var texture=new Texture2D(64,64,TextureFormat.RGBA32,false);
        texture.name="Soft radial particle";
        for(int y=0;y<64;y++)for(int x=0;x<64;x++){
            float r=new Vector2((x-31.5f)/31.5f,(y-31.5f)/31.5f).magnitude;
            float a=Mathf.Pow(Mathf.Clamp01(1-r),2);
            texture.SetPixel(x,y,new Color(1,1,1,a));
        }
        texture.Apply();texture.wrapMode=TextureWrapMode.Clamp;
        AssetDatabase.CreateAsset(texture,"Assets/Msterial/RescueFireParticleTexture.asset");
        mat=new Material(Shader.Find("Universal Render Pipeline/Particles/Unlit"));
        mat.SetTexture("_BaseMap",texture);mat.SetColor("_BaseColor",Color.white);
        mat.SetFloat("_Surface",1);mat.SetFloat("_Blend",0);mat.SetFloat("_ZWrite",0);
        mat.SetFloat("_SrcBlend",(float)UnityEngine.Rendering.BlendMode.SrcAlpha);
        mat.SetFloat("_DstBlend",(float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");mat.renderQueue=3000;
        mat.SetOverrideTag("RenderType","Transparent");
        AssetDatabase.CreateAsset(mat,path);return mat;
    }
    static void Emitter(Transform parent,string name,Material mat,Color color,float rate,float lifetime,float size,float speed,float radius)
    {
        var go=new GameObject(name);go.transform.SetParent(parent,false);
        go.transform.localRotation=Quaternion.Euler(-90,0,0);
        var ps=go.AddComponent<ParticleSystem>();ps.Stop(true,ParticleSystemStopBehavior.StopEmittingAndClear);
        var main=ps.main;main.loop=true;main.playOnAwake=true;main.prewarm=true;
        main.duration=5;main.startLifetime=lifetime;main.startSpeed=new ParticleSystem.MinMaxCurve(speed*.7f,speed);
        main.startSize=new ParticleSystem.MinMaxCurve(size*.6f,size);main.startColor=color;
        main.simulationSpace=ParticleSystemSimulationSpace.World;main.maxParticles=500;
        var emission=ps.emission;emission.rateOverTime=rate;
        var shape=ps.shape;shape.shapeType=ParticleSystemShapeType.Cone;shape.angle=12;shape.radius=radius;
        var colors=ps.colorOverLifetime;colors.enabled=true;
        var gradient=new Gradient();gradient.SetKeys(new[]{new GradientColorKey(Color.white,0),new GradientColorKey(Color.white,1)},
            new[]{new GradientAlphaKey(0,0),new GradientAlphaKey(1,.15f),new GradientAlphaKey(.6f,.6f),new GradientAlphaKey(0,1)});
        colors.color=gradient;
        var sizing=ps.sizeOverLifetime;sizing.enabled=true;
        sizing.size=new ParticleSystem.MinMaxCurve(1,AnimationCurve.Linear(0,1,1,name=="Smoke"?2.2f:.15f));
        var renderer=ps.GetComponent<ParticleSystemRenderer>();renderer.sharedMaterial=mat;
        renderer.shadowCastingMode=UnityEngine.Rendering.ShadowCastingMode.Off;renderer.receiveShadows=false;
        ps.useAutoRandomSeed=false;ps.randomSeed=27001;
    }
    [MenuItem("Tools/Fire Rescue/Install Fire Particle Effects")]
    public static void Install()
    {
        var scene=SceneManager.GetActiveScene();
        if(EditorApplication.isPlayingOrWillChangePlaymode || scene.path!=FurnishedRescueScene.PathName)
            throw new System.InvalidOperationException("Open furnished scene and stop Play first");
        var fire=GameObject.Find("FireSource_01");
        if(fire==null)throw new System.InvalidOperationException("Fire source missing");
        if(GameObject.Find("FireVisualEffects")!=null)return;
        var renderers=fire.GetComponentsInChildren<MeshRenderer>();
        Bounds bounds=new Bounds(fire.transform.position,Vector3.zero);
        foreach(var r in renderers)bounds.Encapsulate(r.bounds);
        var root=new GameObject("FireVisualEffects");Undo.RegisterCreatedObjectUndo(root,"Add fire effects");
        root.transform.position=new Vector3(fire.transform.position.x,Mathf.Max(.05f,bounds.min.y),fire.transform.position.z);
        // Keep world scale independent from the cylinder's non-uniform scale.
        root.transform.SetParent(fire.transform,true);
        var mat=ParticleMaterial();
        float radius=Mathf.Clamp(Mathf.Max(bounds.extents.x,bounds.extents.z)*.65f,.5f,1.5f);
        Emitter(root.transform,"Flames",mat,new Color(1,.12f,.015f,.95f),65,1.2f,1.5f,2.4f,radius);
        Emitter(root.transform,"Embers",mat,new Color(1,.22f,.02f,1),14,1.5f,.13f,3,radius*.7f);
        Emitter(root.transform,"Smoke",mat,new Color(.18f,.19f,.21f,.35f),12,1.6f,1.5f,2,radius);
        foreach(var r in renderers){Undo.RecordObject(r,"Hide fire cylinder visual");r.enabled=false;}
        EditorSceneManager.MarkSceneDirty(scene);EditorSceneManager.SaveScene(scene);AssetDatabase.SaveAssets();
        Debug.Log("FIRE_PARTICLES_INSTALLED; original fire colliders and truth source preserved");
    }
}
