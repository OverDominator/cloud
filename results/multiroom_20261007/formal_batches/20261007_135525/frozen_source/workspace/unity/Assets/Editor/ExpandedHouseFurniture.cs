using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

[InitializeOnLoad]
public static class ExpandedHouseFurniture
{
    static ExpandedHouseFurniture(){EditorApplication.delayCall+=()=>{
        var s=UnityEngine.SceneManagement.SceneManager.GetActiveScene();
        if(!EditorApplication.isPlayingOrWillChangePlaymode && !s.isDirty &&
           s.path==SingleStoreyHouseBuilder.ScenePath && !File.Exists(ScenePath))Build();
    };}
    public const string ScenePath="Assets/Scenes/FireRescue_House_Expanded.unity";
    static Material wood,cloth,steel,paper;
    static Material Mat(string name,Color c){
        string p="Assets/Msterial/Expanded_"+name+".mat";
        var m=AssetDatabase.LoadAssetAtPath<Material>(p);
        if(m==null){m=new Material(Shader.Find("Universal Render Pipeline/Lit"));m.SetColor("_BaseColor",c);AssetDatabase.CreateAsset(m,p);}return m;
    }
    static void Part(Transform t,string name,Vector3 pos,Vector3 size,Material mat){
        var g=GameObject.CreatePrimitive(PrimitiveType.Cube);g.name=name;g.layer=LayerMask.NameToLayer("Environment");
        g.transform.SetParent(t,false);g.transform.localPosition=pos;g.transform.localScale=size;g.GetComponent<Renderer>().sharedMaterial=mat;
    }
    static GameObject Make(string name,float x,float z,string type){
        var g=new GameObject(name);g.transform.position=new Vector3(x,0,z);var t=g.transform;
        if(type=="sofa"){
            Part(t,"Upholstered seat",new Vector3(0,1,0),new Vector3(5,1.1f,2.4f),cloth);
            Part(t,"Backrest",new Vector3(0,2,.95f),new Vector3(5,2,.45f),cloth);
            foreach(float side in new[]{-2.4f,2.4f})Part(t,"Armrest",new Vector3(side,1.4f,0),new Vector3(.4f,1.5f,2.4f),cloth);
        }else if(type=="boxes"){
            for(int i=0;i<3;i++)Part(t,"Cardboard carton",new Vector3((i-1)*1.1f,.6f+(i==1?1.1f:0),0),new Vector3(1.2f,1.2f,1.4f),paper);
        }else{
            float height=type=="low"?1.2f:3.8f;
            Part(t,"Back panel",new Vector3(0,height/2,.7f),new Vector3(3.6f,height,.15f),wood);
            foreach(float side in new[]{-1.75f,1.75f})Part(t,"Side panel",new Vector3(side,height/2,0),new Vector3(.15f,height,1.5f),wood);
            for(float y=.15f;y<=height+.01f;y+=height/3)Part(t,"Shelf",new Vector3(0,y,0),new Vector3(3.6f,.15f,1.5f),wood);
            if(type=="cabinet"){
                foreach(float side in new[]{-.87f,.87f}){
                    Part(t,"Door",new Vector3(side,height/2,-.77f),new Vector3(1.68f,height-.15f,.12f),wood);
                    Part(t,"Handle",new Vector3(side*.2f,height/2,-.87f),new Vector3(.1f,.4f,.1f),steel);
                }
            }else if(type=="books")for(int i=0;i<6;i++)Part(t,"Book",new Vector3(-1.3f+i*.45f,1.8f,0),new Vector3(.28f,.8f,.8f),i%2==0?cloth:paper);
        }return g;
    }
    [MenuItem("Tools/Fire Rescue/Create Expanded Furnished House")]
    public static void Build(){
        if(EditorApplication.isPlayingOrWillChangePlaymode)throw new System.InvalidOperationException("Stop Play first");
        if(File.Exists(ScenePath))throw new System.InvalidOperationException("Expanded scene exists; will not overwrite");
        if(!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())return;
        var scene=EditorSceneManager.OpenScene(SingleStoreyHouseBuilder.ScenePath);
        var scenario=Object.FindFirstObjectByType<HouseFireScenario>();
        if(scenario==null || scenario.fireSources.Length==0)throw new System.InvalidOperationException("House scenario missing");
        EditorSceneManager.SaveScene(scene,ScenePath);
        wood=Mat("Wood",new Color(.35f,.2f,.1f));cloth=Mat("Upholstery",new Color(.2f,.33f,.4f));
        steel=Mat("Steel",new Color(.2f,.22f,.24f));paper=Mat("Cardboard",new Color(.48f,.35f,.2f));
        var furniture=new List<GameObject>(scenario.furniture);
        furniture.Add(Make("Living room sofa",15,-25,"sofa"));
        furniture.Add(Make("TV cabinet",27,-6,"low"));
        furniture.Add(Make("Living room bookcase",15,-5,"books"));
        furniture.Add(Make("Bedroom wardrobe",-15,25,"cabinet"));
        furniture.Add(Make("Bedside cabinet",-28,24,"low"));
        furniture.Add(Make("Office file cabinet",-15,-26,"cabinet"));
        furniture.Add(Make("Storage cardboard stack",15,25,"boxes"));
        furniture.Add(Make("Storage wooden cabinet",27,6,"cabinet"));
        Make("Living room coffee table",21,-26,"low");
        var fires=new List<GameObject>(scenario.fireSources);
        for(int i=4;i<furniture.Count;i++){
            var fire=Object.Instantiate(scenario.fireSources[0]);fire.name="FireSource_Expanded_"+i;
            fire.SetActive(false);var p=furniture[i].transform.position;
            fire.transform.position=new Vector3(p.x,0,p.z);
            fires.Add(fire);
        }
        for(int i=0;i<fires.Count;i++){
            bool broad=furniture[i].name.Contains("sofa") || furniture[i].name.Contains("bed");
            bool boxes=furniture[i].name.Contains("cardboard");
            foreach(var ps in fires[i].GetComponentsInChildren<ParticleSystem>(true)){
                var shape=ps.shape;shape.radius=broad?1.4f:boxes?.65f:.8f;
                var main=ps.main;main.startSpeed=boxes?1.2f:broad?1.6f:2.1f;
                main.startLifetime=1.1f;
                var p=furniture[i].transform.position;ps.transform.position=p+Vector3.up*(boxes?.5f:broad?1.6f:1.2f);
            }
        }
        scenario.furniture=furniture.ToArray();scenario.fireSources=fires.ToArray();
        EditorSceneManager.MarkSceneDirty(scene);EditorSceneManager.SaveScene(scene);AssetDatabase.SaveAssets();
        Debug.Log("EXPANDED_HOUSE_CREATED candidates="+furniture.Count+"; scene review only, navigation not validated");
    }
}
