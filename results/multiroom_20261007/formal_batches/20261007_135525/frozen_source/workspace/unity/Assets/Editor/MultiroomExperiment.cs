using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Independent pilot layout. Does not overwrite either historical scene.
[InitializeOnLoad]
public static class MultiroomExperiment
{
    const string ScenePath = "Assets/Scenes/FireRescue_Multiroom.unity";
    const string Active = "MultiroomPilot.Active";
    static MultiroomExperiment() { EditorApplication.update += Tick; }
    static Material MaterialFor(string name, Color color)
    {
        string path = "Assets/Msterial/" + name + ".mat";
        var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (mat == null)
        {
            mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            AssetDatabase.CreateAsset(mat, path);
        }
        mat.color = color;
        return mat;
    }
    static void Block(Transform parent, string name, Vector3 pos, Vector3 size, Material mat)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = name; go.transform.parent = parent;
        go.transform.position = pos; go.transform.localScale = size;
        go.layer = LayerMask.NameToLayer("Environment");
        go.GetComponent<Renderer>().sharedMaterial = mat;
    }
    [MenuItem("Tools/Fire Rescue/Build Multiroom Pilot")]
    public static void Build()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Stop play first");
        var scene = EditorSceneManager.OpenScene("Assets/Scenes/FireRescue_Easy.unity");
        // First create a separate scene file, then edit that copy.
        if (!EditorSceneManager.SaveScene(scene, ScenePath))
            throw new IOException("Cannot save pilot scene");
        var old = GameObject.Find("SimplifiedFireScene");
        if (old != null) UnityEngine.Object.DestroyImmediate(old);
        var host = new GameObject("MultiroomEnvironment");
        var wall = MaterialFor("Multiroom_Wall", new Color(.75f,.78f,.80f));
        var floor = MaterialFor("Multiroom_Floor", new Color(.36f,.40f,.43f));
        var furniture = MaterialFor("Multiroom_Furniture", new Color(.32f,.27f,.20f));
        Block(host.transform,"Floor",new Vector3(0,-.25f,0),new Vector3(65,.5f,61),floor);
        foreach (float z in new[]{-30f,30f})
            Block(host.transform,"Boundary_Z_"+z,new Vector3(0,2.5f,z),new Vector3(65,5,1),wall);
        foreach (float x in new[]{-32f,32f})
            Block(host.transform,"Boundary_X_"+x,new Vector3(x,2.5f,0),new Vector3(1,5,61),wall);
        // Central corridor: x=-8..8. Four 10-unit clear door gaps at z=+/-15.
        foreach (float x in new[]{-8f,8f})
        {
            foreach (float z in new[]{-25f,0f,25f})
                Block(host.transform,"RoomFront_"+x+"_"+z,new Vector3(x,2.5f,z),
                    new Vector3(1,5,z==0 ? 20:10),wall);
            Block(host.transform,"RoomDivider_"+x,new Vector3(Mathf.Sign(x)*20,2.5f,0),new Vector3(24,5,1),wall);
        }
        // Sparse furnishings kept away from door approaches.
        foreach(float x in new[]{-25f,25f})
            foreach(float z in new[]{-24f,24f})
                Block(host.transform,"Cabinet_"+x+"_"+z,new Vector3(x,1.5f,z),new Vector3(5,3,2),furniture);
        var robot=GameObject.Find("TrackedRobot");
        robot.transform.SetPositionAndRotation(new Vector3(0,0,-24),Quaternion.identity);
        var first=GameObject.Find("Victim_01");
        var second=GameObject.Find("Victim_02");
        var victims=new[]{first,second,UnityEngine.Object.Instantiate(first),UnityEngine.Object.Instantiate(first)};
        var positions=new[]{new Vector3(-22,0,-15),new Vector3(22,0,-15),new Vector3(-22,0,15),new Vector3(22,0,15)};
        for(int i=0;i<4;i++)
        {
            victims[i].name="Victim_0"+(i+1);
            var position=positions[i];
            position.y=victims[i].transform.position.y;
            victims[i].transform.position=position;
        }
        var fire=GameObject.Find("FireSource_01");
        if(fire!=null) fire.transform.position=new Vector3(-27,fire.transform.position.y,6);
        var sensor=UnityEngine.Object.FindFirstObjectByType<VictimCallSensor>();
        if(sensor==null) sensor=robot.AddComponent<VictimCallSensor>();
        sensor.acousticSeed=27001;
        Physics.SyncTransforms();
        var cols=robot.GetComponentsInChildren<Collider>().Where(c=>c.enabled && !c.isTrigger).ToArray();
        Bounds bounds=cols[0].bounds;
        foreach(var c in cols) bounds.Encapsulate(c.bounds);
        Debug.Log("MULTIROOM_ROBOT_BOUNDS "+bounds);
        Directory.CreateDirectory("PilotEvidence");
        File.WriteAllText("PilotEvidence/geometry.txt","Robot collider union: "+bounds+"\nDoors: 10 scene units; corridor clear width: 15.\nPilot only; no formal results.");
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene,ScenePath);
        AssetDatabase.SaveAssets();
        Debug.Log("MULTIROOM_BUILD_OK");
    }
    public static void Run()
    {
        int seconds=240;
        var args=Environment.GetCommandLineArgs();
        for(int i=0;i+1<args.Length;i++)
            if(args[i]=="-pilotSeconds") seconds=int.Parse(args[i+1]);
        RunFor(seconds);
    }
    public static void RunFor(int seconds)
    {
        if(!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
        if(!File.Exists(ExpandedHouseFurniture.ScenePath))
            throw new FileNotFoundException("Create the expanded furnished house before starting",ExpandedHouseFurniture.ScenePath);
        EditorSceneManager.OpenScene(ExpandedHouseFurniture.ScenePath);
        Debug.Log("PILOT_ACTIVE_SCENE "+ExpandedHouseFurniture.ScenePath);
        SessionState.SetBool(Active,true);
        SessionState.SetString(Active+".deadline",DateTime.UtcNow.AddSeconds(seconds).ToString("O"));
        EditorApplication.isPlaying=true;
        if(!Application.isBatchMode)
            EditorApplication.ExecuteMenuItem("Window/General/Game");
    }
    public static void Capture()
    {
        EditorSceneManager.OpenScene(ScenePath);
        Directory.CreateDirectory("PilotEvidence");
        var go=new GameObject("TemporaryPilotOverview");
        var camera=go.AddComponent<Camera>();
        camera.enabled=false;
        camera.cullingMask=~0;
        camera.useOcclusionCulling=false;
        camera.nearClipPlane=.1f;
        camera.farClipPlane=1000f;
        ShaderUtil.allowAsyncCompilation=false;
        camera.transform.position=new Vector3(0,100,-70);
        camera.transform.LookAt(Vector3.zero);
        camera.orthographic=true; camera.orthographicSize=35;
        camera.backgroundColor=Color.white; camera.clearFlags=CameraClearFlags.SolidColor;
        var rt=new RenderTexture(1400,1400,24);
        Physics.SyncTransforms();
        camera.targetTexture=rt; camera.Render();
        RenderTexture.active=rt;
        var image=new Texture2D(1400,1400,TextureFormat.RGB24,false);
        image.ReadPixels(new Rect(0,0,1400,1400),0,0); image.Apply();
        File.WriteAllBytes("PilotEvidence/layout_overview.png",image.EncodeToPNG());
        RenderTexture.active=null; camera.targetTexture=null;
        UnityEngine.Object.DestroyImmediate(image); UnityEngine.Object.DestroyImmediate(rt);
        UnityEngine.Object.DestroyImmediate(go);
        Debug.Log("MULTIROOM_CAPTURE_OK");
    }
    static void Tick()
    {
        if(!SessionState.GetBool(Active,false)) return;
        var deadline=DateTime.Parse(SessionState.GetString(Active+".deadline",""),null,System.Globalization.DateTimeStyles.RoundtripKind);
        if(DateTime.UtcNow<deadline && !File.Exists("PilotEvidence/stop")) return;
        SessionState.SetBool(Active,false);
        if(Application.isBatchMode) EditorApplication.Exit(0);
        else EditorApplication.isPlaying=false; // Keep the visible editor open for inspection.
    }
}
