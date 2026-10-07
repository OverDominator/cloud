using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System.IO;

public static class SingleStoreyHouseBuilder
{
    public const string ScenePath="Assets/Scenes/FireRescue_House.unity";
    static void Part(Transform parent,string name,Vector3 p,Vector3 s,Color color)
    {
        var g=GameObject.CreatePrimitive(PrimitiveType.Cube);g.name=name;g.layer=LayerMask.NameToLayer("Environment");
        g.transform.SetParent(parent,false);g.transform.localPosition=p;g.transform.localScale=s;
        string path="Assets/Msterial/House_"+name.Replace(" ","_")+".mat";
        var mat=AssetDatabase.LoadAssetAtPath<Material>(path);
        if(mat==null){mat=new Material(Shader.Find("Universal Render Pipeline/Lit"));mat.SetColor("_BaseColor",color);AssetDatabase.CreateAsset(mat,path);}
        g.GetComponent<Renderer>().sharedMaterial=mat;
    }
    [MenuItem("Tools/Fire Rescue/Create Single-storey House")]
    public static void Build()
    {
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new System.InvalidOperationException("Stop Play first");
        if(File.Exists(ScenePath))throw new System.InvalidOperationException("House scene already exists; preserve manual edits");
        if(!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())return;
        var scene=EditorSceneManager.OpenScene(FurnishedRescueScene.PathName);
        var original=GameObject.Find("FireSource_01");
        if(original==null || original.GetComponentInChildren<ParticleSystem>()==null)
            throw new System.InvalidOperationException("Install particle fire in furnished scene first");
        EditorSceneManager.SaveScene(scene,ScenePath);
        var bed=new GameObject("Bedroom bed");bed.transform.position=new Vector3(-24,0,24);
        Part(bed.transform,"Bed frame",new Vector3(0,.55f,0),new Vector3(5,1.1f,6),new Color(.28f,.15f,.09f));
        Part(bed.transform,"Mattress",new Vector3(0,1.3f,0),new Vector3(4.8f,.5f,5.8f),new Color(.82f,.8f,.72f));
        Part(bed.transform,"Blanket",new Vector3(0,1.6f,-.6f),new Vector3(4.9f,.15f,4.4f),new Color(.17f,.35f,.45f));
        Part(bed.transform,"Pillow",new Vector3(0,1.7f,2),new Vector3(3,.35f,1.2f),Color.white);
        Part(bed.transform,"Headboard",new Vector3(0,1.6f,3),new Vector3(5,.0f+3.2f,.2f),new Color(.28f,.15f,.09f));
        var bench=GameObject.Find("Laboratory bench");if(bench!=null)Object.DestroyImmediate(bench);
        var scenario=new GameObject("House scenario - seed and fire count").AddComponent<HouseFireScenario>();
        scenario.furniture=new[]{GameObject.Find("Office desk"),GameObject.Find("Meeting table"),bed,GameObject.Find("Storage shelving")};
        foreach(var item in scenario.furniture)if(item==null)throw new System.InvalidOperationException("Missing candidate furniture");
        scenario.fireSources=new GameObject[scenario.furniture.Length];
        original.SetActive(false);
        for(int i=0;i<scenario.furniture.Length;i++){
            var fire=Object.Instantiate(original);fire.name="FireSource_House_"+i;
            Vector3 target=scenario.furniture[i].transform.position;
            fire.transform.position=new Vector3(target.x,original.transform.position.y,target.z);
            foreach(var t in fire.GetComponentsInChildren<Transform>(true))
                if(t.name=="BurningCabinetVisual")t.gameObject.SetActive(false);
            scenario.fireSources[i]=fire;
        }
        Object.DestroyImmediate(original);
        scenario.gameObject.AddComponent<SimulatedFireTruthPublisher>();
        EditorSceneManager.MarkSceneDirty(scene);EditorSceneManager.SaveScene(scene);AssetDatabase.SaveAssets();
        Debug.Log("HOUSE_SCENE_CREATED "+ScenePath);
    }
}
