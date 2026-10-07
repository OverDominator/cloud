using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.Rendering.Universal;

// Object-class pseudo-thermal rendering, NOT temperature or radiometry.
// Changes exist only during this synchronous camera render and are restored even on error.
public static class SemanticThermalRender
{
    static Material grey,yellow,red;
    struct Saved { public Renderer r; public bool enabled; public Material[] materials; }
    static Material Make(Color color)
    {
        var shader=Shader.Find("Universal Render Pipeline/Unlit");
        if(shader==null)throw new InvalidOperationException("Semantic unlit shader missing");
        var m=new Material(shader){hideFlags=HideFlags.HideAndDontSave};
        m.SetColor("_BaseColor",color);return m;
    }
    static int ClassOf(Transform t)
    {
        bool victim=false;
        while(t!=null){
            if(HouseFireScenario.BurningFurniture.Contains(t))return 2;
            if(t.CompareTag("FireSource"))return 2;
            if(t.name.StartsWith("Victim_") || t.CompareTag("Victim"))victim=true;
            t=t.parent;
        }
        return victim?1:0;
    }
    public static bool Applies => SceneManager.GetActiveScene().name=="FireRescue_House_Expanded" || SceneManager.GetActiveScene().name=="FireRescue_House" ||
        SceneManager.GetActiveScene().name=="FireRescue_Furnished";
    public static void Render(Camera camera)
    {
        if(!Applies){camera.Render();return;}
        if(grey==null){grey=Make(new Color(.4f,.4f,.4f));yellow=Make(new Color(1,220f/255f,0));red=Make(Color.red);}
        var saved=new List<Saved>();
        var flags=camera.clearFlags;var background=camera.backgroundColor;
        var data=camera.GetComponent<UniversalAdditionalCameraData>();
        bool post=data!=null && data.renderPostProcessing;
        try {
            foreach(var r in UnityEngine.Object.FindObjectsByType<Renderer>(FindObjectsSortMode.None)){
                if(!r.gameObject.activeInHierarchy)continue;
                saved.Add(new Saved{r=r,enabled=r.enabled,materials=r.sharedMaterials});
                // Particles and diagnostic lines are presentation, not sensor evidence.
                if(!(r is MeshRenderer) && !(r is SkinnedMeshRenderer)){r.enabled=false;continue;}
                int cls=ClassOf(r.transform);
                // Preserve hidden legacy person meshes. Restore active fire's original
                // geometric proxy even though its cylinder is hidden in the display view.
                if(!r.enabled && cls!=2)continue;
                if(cls==2)r.enabled=true;
                var mats=new Material[r.sharedMaterials.Length];
                for(int i=0;i<mats.Length;i++)mats[i]=cls==2?red:cls==1?yellow:grey;
                r.sharedMaterials=mats;
            }
            camera.clearFlags=CameraClearFlags.SolidColor;camera.backgroundColor=new Color(.4f,.4f,.4f);
            if(data!=null)data.renderPostProcessing=false;
            camera.Render();
        } finally {
            foreach(var item in saved)if(item.r!=null){item.r.sharedMaterials=item.materials;item.r.enabled=item.enabled;}
            camera.clearFlags=flags;camera.backgroundColor=background;
            if(data!=null)data.renderPostProcessing=post;
        }
    }
}
