using System;
using System.IO;
using UnityEngine;

// Fixed-at-start scenario selection, not a combustion/spread simulation.
[DefaultExecutionOrder(-1000)]
public class HouseFireScenario : MonoBehaviour
{
    public static readonly System.Collections.Generic.HashSet<Transform> BurningFurniture = new System.Collections.Generic.HashSet<Transform>();
    public int seed=27001;
    [Range(.3f,1f)] public float flameCoverage=.8f;
    [Range(.5f,2f)] public float flameIntensity=1f;
    [Range(1,4)] public int fireCount=2;
    public GameObject[] furniture;
    public GameObject[] fireSources;
    [Serializable] class Selection { public int seed; public int count; public string[] furniture; public Vector3[] positions; }
    void Awake()
    {
        BurningFurniture.Clear();
        if(furniture.Length!=fireSources.Length || furniture.Length==0){Debug.LogError("Invalid house fire candidates");return;}
        var order=new int[furniture.Length];for(int i=0;i<order.Length;i++){order[i]=i;fireSources[i].SetActive(false);}
        var rng=new System.Random(seed);
        for(int i=order.Length-1;i>0;i--){int j=rng.Next(i+1);int v=order[i];order[i]=order[j];order[j]=v;}
        int n=Mathf.Clamp(fireCount,0,order.Length);
        var record=new Selection{seed=seed,count=n,furniture=new string[n],positions=new Vector3[n]};
        for(int k=0;k<n;k++){
            int i=order[k];
            FurnitureFireVisualFit.Apply(furniture[i],fireSources[i],flameCoverage,flameIntensity);
            fireSources[i].SetActive(true);
            BurningFurniture.Add(furniture[i].transform);
            foreach(var r in furniture[i].GetComponentsInChildren<MeshRenderer>()){
                var materials=r.materials;
                foreach(var m in materials)if(m.HasProperty("_BaseColor"))m.SetColor("_BaseColor",new Color(.055f,.04f,.03f));
            }
            record.furniture[k]=furniture[i].name;record.positions[k]=fireSources[i].transform.position;
        }
        string dir=Path.Combine(Application.dataPath,"../PilotEvidence");Directory.CreateDirectory(dir);
        File.WriteAllText(Path.Combine(dir,"house_fire_"+DateTime.UtcNow.ToString("yyyyMMdd_HHmmss_fff")+".json"),JsonUtility.ToJson(record,true));
        Debug.Log("HOUSE_FIRE_SELECTION "+JsonUtility.ToJson(record));
    }
}
