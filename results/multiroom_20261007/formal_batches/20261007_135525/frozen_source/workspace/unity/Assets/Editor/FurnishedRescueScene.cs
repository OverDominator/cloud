using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System.IO;

// Procedural, editable furniture. Independent scene; no downloaded assets.
public static class FurnishedRescueScene
{
    public const string PathName="Assets/Scenes/FireRescue_Furnished.unity";
    static Material wood, metal, fabric, screen;
    static Material Mat(string name, Color color)
    {
        string path="Assets/Msterial/Furnished_"+name+".mat";
        var m=AssetDatabase.LoadAssetAtPath<Material>(path);
        if(m==null){m=new Material(Shader.Find("Universal Render Pipeline/Lit"));AssetDatabase.CreateAsset(m,path);}
        m.SetColor("_BaseColor",color); m.color=color; return m;
    }
    static void Part(Transform parent,string name,Vector3 p,Vector3 size,Material material)
    {
        var g=GameObject.CreatePrimitive(PrimitiveType.Cube);g.name=name;
        g.transform.SetParent(parent,false);g.transform.localPosition=p;g.transform.localScale=size;
        g.layer=LayerMask.NameToLayer("Environment");g.GetComponent<Renderer>().sharedMaterial=material;
    }
    static Transform Group(Transform parent,string name,float x,float z)
    {
        var g=new GameObject(name);g.transform.SetParent(parent,false);g.transform.localPosition=new Vector3(x,0,z);return g.transform;
    }
    static void Desk(Transform parent,string name,float x,float z,bool computer)
    {
        var t=Group(parent,name,x,z);
        Part(t,"Wood desktop",new Vector3(0,2.4f,0),new Vector3(6,.3f,3),wood);
        foreach(float a in new[]{-2.6f,2.6f})foreach(float b in new[]{-1.1f,1.1f})
            Part(t,"Steel leg",new Vector3(a,1.15f,b),new Vector3(.22f,2.3f,.22f),metal);
        if(computer){
            Part(t,"Monitor",new Vector3(0,3.5f,.6f),new Vector3(2.3f,1.4f,.18f),screen);
            Part(t,"Monitor stand",new Vector3(0,2.85f,.6f),new Vector3(.2f,.7f,.2f),metal);
            Part(t,"Keyboard",new Vector3(0,2.6f,-.6f),new Vector3(1.6f,.1f,.5f),metal);
        }
    }
    static void Chair(Transform parent,float x,float z)
    {
        var t=Group(parent,"Chair",x,z);
        Part(t,"Seat",new Vector3(0,1.35f,0),new Vector3(1.8f,.25f,1.8f),fabric);
        Part(t,"Backrest",new Vector3(0,2.25f,-.8f),new Vector3(1.8f,1.7f,.22f),fabric);
        foreach(float a in new[]{-.65f,.65f})foreach(float b in new[]{-.65f,.65f})
            Part(t,"Leg",new Vector3(a,.65f,b),new Vector3(.16f,1.3f,.16f),metal);
    }
    static void Shelf(Transform parent,float x,float z)
    {
        var t=Group(parent,"Storage shelving",x,z);
        foreach(float a in new[]{-2.8f,2.8f})Part(t,"Upright",new Vector3(a,2.5f,0),new Vector3(.2f,5,2),metal);
        foreach(float h in new[]{.3f,1.8f,3.3f,4.8f}){
            Part(t,"Shelf",new Vector3(0,h,0),new Vector3(5.8f,.15f,2),wood);
            if(h<4)Part(t,"Storage box",new Vector3(-1,h+.55f,0),new Vector3(1.5f,1,1.4f),fabric);
        }
    }
    [MenuItem("Tools/Fire Rescue/Create Furnished Scene")]
    public static void Build()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new System.InvalidOperationException("Stop Play first");
        if(File.Exists(PathName))throw new System.InvalidOperationException("Furnished scene exists; preserve edits rather than overwrite");
        if(!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())return;
        var scene=EditorSceneManager.OpenScene("Assets/Scenes/FireRescue_Multiroom.unity");
        EditorSceneManager.SaveScene(scene,PathName);
        foreach(var t in Object.FindObjectsByType<Transform>(FindObjectsSortMode.None))
            if(t.name.StartsWith("Cabinet_"))Object.DestroyImmediate(t.gameObject);
        wood=Mat("wood",new Color(.48f,.27f,.12f));metal=Mat("steel",new Color(.15f,.18f,.21f));
        fabric=Mat("blue",new Color(.12f,.33f,.48f));screen=Mat("screen",new Color(.025f,.07f,.1f));
        var root=new GameObject("Indoor furniture - oversized simulation scale").transform;
        Desk(root,"Office desk",-24,-24,true);Chair(root,-24,-27);
        Desk(root,"Meeting table",24,-24,false);Chair(root,22,-27);Chair(root,26,-27);
        Desk(root,"Laboratory bench",-24,24,true);Chair(root,-24,21);
        Shelf(root,24,25);Shelf(root,24,21);
        Physics.SyncTransforms();EditorSceneManager.MarkSceneDirty(scene);EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();Debug.Log("FURNISHED_SCENE_CREATED "+PathName);
    }
}
