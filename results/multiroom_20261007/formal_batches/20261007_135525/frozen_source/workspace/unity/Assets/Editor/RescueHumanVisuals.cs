using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using System.Linq;

// Stylised human appearance; existing victim identity and colliders remain intact.
public static class RescueHumanVisuals
{
    // The building is deliberately oversized for the existing robot.
    // Keep a person below the 5-unit ceiling rather than using the old tiny marker height.
    const float PersonHeight = 4.2f;
    static void Resize(Transform host)
    {
        var renderers=host.GetComponentsInChildren<MeshRenderer>();
        if(renderers.Length==0)return;
        Bounds b=renderers[0].bounds;foreach(var r in renderers)b.Encapsulate(r.bounds);
        if(b.size.y<.001f)return;
        Undo.RecordObject(host,"Correct person visual scale");
        float foot=b.min.y;
        host.localScale*=PersonHeight/b.size.y;
        Bounds after=renderers[0].bounds;foreach(var r in renderers)after.Encapsulate(r.bounds);
        host.position+=Vector3.up*(foot-after.min.y);
    }
    static Material Mat(string name,Color color)
    {
        string path="Assets/Msterial/Human_"+name+".mat";
        var mat=AssetDatabase.LoadAssetAtPath<Material>(path);
        if(mat==null){mat=new Material(Shader.Find("Universal Render Pipeline/Lit"));
            mat.SetColor("_BaseColor",color);AssetDatabase.CreateAsset(mat,path);}
        return mat;
    }
    static void Part(Transform root,string name,PrimitiveType type,Vector3 p,Vector3 s,Material mat,float tilt=0)
    {
        var obj=GameObject.CreatePrimitive(type);obj.name=name;
        obj.transform.SetParent(root,false);obj.transform.localPosition=p;
        obj.transform.localScale=s;obj.transform.localRotation=Quaternion.Euler(0,0,tilt);
        obj.GetComponent<Renderer>().sharedMaterial=mat;
        Object.DestroyImmediate(obj.GetComponent<Collider>());
        obj.layer=root.gameObject.layer;
    }
    [MenuItem("Tools/Fire Rescue/Upgrade Victim Human Appearance")]
    public static void Install()
    {
        var scene=SceneManager.GetActiveScene();
        if(EditorApplication.isPlayingOrWillChangePlaymode ||
           (scene.path!=FurnishedRescueScene.PathName && scene.path!=SingleStoreyHouseBuilder.ScenePath))
            throw new System.InvalidOperationException("Open furnished or house scene and stop Play first");
        var skin=Mat("Skin",new Color(.57f,.32f,.20f));
        var jacket=Mat("YellowJacket",new Color(1,.86f,0));
        var pants=Mat("Trousers",new Color(.12f,.18f,.24f));
        var dark=Mat("HairAndBoots",new Color(.035f,.025f,.02f));
        var white=Mat("Eyes",new Color(.8f,.8f,.75f));
        int count=0;
        foreach(var victim in scene.GetRootGameObjects().Where(g=>g.name.StartsWith("Victim_")))
        {
            var existing=victim.transform.Find("HumanAppearance");
            if(existing!=null){Resize(existing);count++;continue;}
            var old=victim.GetComponentsInChildren<MeshRenderer>().Where(r=>r.enabled &&
                (r.name=="Head" || r.name=="Body")).ToArray();
            if(old.Length==0){Debug.LogWarning("No original head/body: "+victim.name);continue;}
            var bounds=old[0].bounds;foreach(var r in old)bounds.Encapsulate(r.bounds);
            var host=new GameObject("HumanAppearance");host.layer=old[0].gameObject.layer;
            Undo.RegisterCreatedObjectUndo(host,"Human victim appearance");
            host.transform.position=new Vector3(bounds.center.x,bounds.min.y,bounds.center.z);
            host.transform.rotation=victim.transform.rotation;
            // Standard 1.8-unit human mapped to the original world-space visual height.
            host.transform.localScale=Vector3.one*(bounds.size.y/1.8f);
            host.transform.SetParent(victim.transform,true);
            var t=host.transform;
            Part(t,"Jacket torso",PrimitiveType.Capsule,new Vector3(0,1.13f,0),new Vector3(.46f,.31f,.29f),jacket);
            Part(t,"Neck",PrimitiveType.Cylinder,new Vector3(0,1.48f,0),new Vector3(.12f,.06f,.12f),skin);
            Part(t,"Head",PrimitiveType.Sphere,new Vector3(0,1.64f,0),new Vector3(.27f,.32f,.26f),skin);
            Part(t,"Hair",PrimitiveType.Sphere,new Vector3(0,1.735f,-.025f),new Vector3(.28f,.13f,.26f),dark);
            Part(t,"Nose",PrimitiveType.Sphere,new Vector3(0,1.64f,.14f),new Vector3(.055f,.07f,.065f),skin);
            foreach(float side in new[]{-1f,1f}){
                Part(t,"Sleeve",PrimitiveType.Capsule,new Vector3(side*.29f,1.16f,0),new Vector3(.15f,.21f,.16f),jacket,side*12);
                Part(t,"Forearm",PrimitiveType.Capsule,new Vector3(side*.35f,.87f,.025f),new Vector3(.12f,.16f,.12f),jacket,side*5);
                Part(t,"Hand",PrimitiveType.Sphere,new Vector3(side*.37f,.69f,.035f),new Vector3(.12f,.17f,.1f),skin);
                Part(t,"Trouser leg",PrimitiveType.Capsule,new Vector3(side*.13f,.46f,0),new Vector3(.19f,.35f,.2f),pants);
                Part(t,"Boot",PrimitiveType.Cube,new Vector3(side*.13f,.08f,.07f),new Vector3(.19f,.16f,.32f),dark);
                Part(t,"Eye",PrimitiveType.Sphere,new Vector3(side*.055f,1.685f,.117f),new Vector3(.045f,.03f,.022f),white);
                Part(t,"Pupil",PrimitiveType.Sphere,new Vector3(side*.055f,1.685f,.129f),new Vector3(.018f,.02f,.009f),dark);
            }
            foreach(var r in old){Undo.RecordObject(r,"Hide original victim mesh");r.enabled=false;}
            Resize(host.transform);
            count++;
        }
        EditorSceneManager.MarkSceneDirty(scene);EditorSceneManager.SaveScene(scene);AssetDatabase.SaveAssets();
        Debug.Log("HUMAN_VISUALS_INSTALLED count="+count+"; identity and colliders unchanged");
    }
}
