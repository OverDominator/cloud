using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

[InitializeOnLoad]
public static class RescueBurningCabinet
{
    static RescueBurningCabinet(){EditorApplication.delayCall+=AutoInstall;}
    static void AutoInstall()
    {
        var s=SceneManager.GetActiveScene();
        if(!EditorApplication.isPlayingOrWillChangePlaymode && !s.isDirty &&
           s.path==FurnishedRescueScene.PathName && GameObject.Find("FireVisualEffects")!=null &&
           GameObject.Find("BurningCabinetVisual")==null)Install();
    }
    static Material Mat(string name,Color color)
    {
        string path="Assets/Msterial/"+name+".mat";
        var m=AssetDatabase.LoadAssetAtPath<Material>(path);
        if(m==null){m=new Material(Shader.Find("Universal Render Pipeline/Lit"));
            m.SetColor("_BaseColor",color);AssetDatabase.CreateAsset(m,path);}
        return m;
    }
    static void Panel(Transform parent,string name,Vector3 pos,Vector3 size,Material mat)
    {
        var g=GameObject.CreatePrimitive(PrimitiveType.Cube);g.name=name;
        g.transform.SetParent(parent,false);g.transform.localPosition=pos;g.transform.localScale=size;
        g.GetComponent<Renderer>().sharedMaterial=mat;
        // Visual-only carrier: the existing fire hazard collider remains authoritative.
        Object.DestroyImmediate(g.GetComponent<Collider>());
    }
    [MenuItem("Tools/Fire Rescue/Install Burning Cabinet")]
    public static void Install()
    {
        var scene=SceneManager.GetActiveScene();
        if(EditorApplication.isPlayingOrWillChangePlaymode || scene.path!=FurnishedRescueScene.PathName)
            throw new System.InvalidOperationException("Open furnished scene and stop Play first");
        var fire=GameObject.Find("FireSource_01");var fx=GameObject.Find("FireVisualEffects");
        if(fire==null || fx==null)throw new System.InvalidOperationException("Install fire particles first");
        if(GameObject.Find("BurningCabinetVisual")!=null)return;
        var root=new GameObject("BurningCabinetVisual");Undo.RegisterCreatedObjectUndo(root,"Add burning cabinet");
        root.transform.position=new Vector3(fire.transform.position.x,fx.transform.position.y,fire.transform.position.z);
        root.transform.SetParent(fire.transform,true);
        var wood=Mat("BurningCabinetWood",new Color(.23f,.11f,.055f));
        var charred=Mat("BurningCabinetChar",new Color(.045f,.035f,.028f));
        var metal=Mat("BurningCabinetHandles",new Color(.2f,.21f,.22f));
        Panel(root.transform,"Left side",new Vector3(-1.1f,1.15f,0),new Vector3(.15f,2.3f,1.6f),wood);
        Panel(root.transform,"Right side",new Vector3(1.1f,1.15f,0),new Vector3(.15f,2.3f,1.6f),wood);
        Panel(root.transform,"Scorched back",new Vector3(0,1.15f,.75f),new Vector3(2.2f,2.3f,.12f),charred);
        foreach(float h in new[]{.12f,1f,2.25f})
            Panel(root.transform,"Charred shelf",new Vector3(0,h,0),new Vector3(2.2f,.12f,1.6f),charred);
        Panel(root.transform,"Remaining lower door",new Vector3(-.55f,.5f,-.8f),new Vector3(1f,.85f,.12f),wood);
        Panel(root.transform,"Door handle",new Vector3(-.15f,.55f,-.89f),new Vector3(.07f,.28f,.08f),metal);
        // Place emitters above the middle shelf rather than underneath the cabinet.
        foreach(var ps in fx.GetComponentsInChildren<ParticleSystem>()){
            Undo.RecordObject(ps.transform,"Align cabinet fire");
            ps.transform.position=root.transform.position+new Vector3(0,1.15f,-.15f);
        }
        EditorSceneManager.MarkSceneDirty(scene);EditorSceneManager.SaveScene(scene);AssetDatabase.SaveAssets();
        Debug.Log("BURNING_CABINET_INSTALLED: visual carrier only; existing hazard collider unchanged");
    }
}
