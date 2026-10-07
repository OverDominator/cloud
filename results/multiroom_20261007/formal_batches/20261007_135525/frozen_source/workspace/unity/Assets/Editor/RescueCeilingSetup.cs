using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

[InitializeOnLoad]
public static class RescueCeilingSetup
{
    const string LayerName="RobotVisibleCeiling";
    static RescueCeilingSetup(){ EditorApplication.delayCall += AutoInstall; }
    static void AutoInstall()
    {
        var s=SceneManager.GetActiveScene();
        if(!EditorApplication.isPlayingOrWillChangePlaymode && !s.isDirty &&
           s.path==FurnishedRescueScene.PathName && GameObject.Find("RobotVisibleCeiling")==null)
            Install();
    }
    [MenuItem("Tools/Fire Rescue/Install Robot-visible Ceiling")]
    public static void Install()
    {
        var scene=SceneManager.GetActiveScene();
        if(EditorApplication.isPlayingOrWillChangePlaymode || scene.path!=FurnishedRescueScene.PathName)
            throw new System.InvalidOperationException("Open furnished scene and stop Play first.");
        var tags=new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0]);
        var layers=tags.FindProperty("layers");
        int layer=LayerMask.NameToLayer(LayerName);
        if(layer<0){
            for(int i=8;i<32;i++)if(string.IsNullOrEmpty(layers.GetArrayElementAtIndex(i).stringValue)){
                layers.GetArrayElementAtIndex(i).stringValue=LayerName;layer=i;break;
            }
            if(layer<0)throw new System.InvalidOperationException("No free user layer");
            tags.ApplyModifiedProperties();
        }
        var ceiling=GameObject.Find("RobotVisibleCeiling");
        if(ceiling==null){
            ceiling=GameObject.CreatePrimitive(PrimitiveType.Cube);
            ceiling.name="RobotVisibleCeiling";
            Undo.RegisterCreatedObjectUndo(ceiling,"Add ceiling");
            ceiling.transform.position=new Vector3(0,5.15f,0);
            ceiling.transform.localScale=new Vector3(65,.3f,61);
            var material=new Material(Shader.Find("Universal Render Pipeline/Lit"));
            material.SetColor("_BaseColor",new Color(.85f,.86f,.88f));
            const string path="Assets/Msterial/RobotVisibleCeiling.mat";
            var existing=AssetDatabase.LoadAssetAtPath<Material>(path);
            if(existing==null)AssetDatabase.CreateAsset(material,path);
            else {Object.DestroyImmediate(material);material=existing;}
            ceiling.GetComponent<Renderer>().sharedMaterial=material;
        }
        ceiling.layer=layer;
        var robot=GameObject.Find("TrackedRobot");
        foreach(var camera in Object.FindObjectsByType<Camera>(FindObjectsInactive.Include,FindObjectsSortMode.None)){
            Undo.RecordObject(camera,"Ceiling camera mask");
            bool robotView=robot!=null && camera.transform.IsChildOf(robot.transform);
            foreach(var publisher in Object.FindObjectsByType<ThermalImagePublisher>(FindObjectsInactive.Include,FindObjectsSortMode.None))
                if(publisher.thermalCamera==camera)robotView=true;
            if(robotView)camera.cullingMask|=1<<layer;
            else camera.cullingMask&=~(1<<layer);
        }
        // Scene-only visibility; does not disable renderer or collider.
        Tools.visibleLayers &= ~(1<<layer);
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();
        Debug.Log("ROBOT_CEILING_INSTALLED layer="+layer+" underside_y=5; collider retained; LiDAR mask unchanged");
    }
}
