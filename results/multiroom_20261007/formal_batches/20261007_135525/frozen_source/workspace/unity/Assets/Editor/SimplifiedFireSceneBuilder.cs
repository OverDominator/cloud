using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static partial class SimplifiedFireSceneBuilder
{
    private const string SourceScenePath = "Assets/Scenes/SampleScene.unity";
    private const string OutputScenePath = "Assets/Scenes/FireRescue_Simplified.unity";
    private const string MaterialFolder = "Assets/Msterial/SimplifiedScene";

    [MenuItem("Tools/Fire Rescue/Create Simplified Experiment Scene")]
    public static void CreateScene()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            EditorUtility.DisplayDialog(
                "Fire Rescue",
                "Stop Play Mode before rebuilding the scene.",
                "OK");
            return;
        }

        if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
        {
            return;
        }

        Scene source = EditorSceneManager.OpenScene(SourceScenePath, OpenSceneMode.Single);
        if (!source.IsValid())
        {
            throw new InvalidOperationException("Unable to open " + SourceScenePath);
        }

        if (!EditorSceneManager.SaveScene(source, OutputScenePath, true))
        {
            throw new InvalidOperationException("Unable to create " + OutputScenePath);
        }

        Scene scene = SceneManager.GetActiveScene();
        RemoveOldEnvironment(scene);

        int environmentLayer = LayerMask.NameToLayer("Environment");
        if (environmentLayer < 0)
        {
            environmentLayer = 0;
            Debug.LogWarning("Environment layer does not exist; using Default.");
        }

        Material floorMaterial = GetOrCreateMaterial(
            "Floor_Grey", new Color(0.42f, 0.44f, 0.47f));
        Material wallMaterial = GetOrCreateMaterial(
            "Wall_LightGrey", new Color(0.72f, 0.73f, 0.75f));
        Material obstacleMaterial = GetOrCreateMaterial(
            "Obstacle_DarkGrey", new Color(0.25f, 0.27f, 0.30f));

        GameObject environment = new GameObject("SimplifiedFireScene");
        environment.AddComponent<SimpleSceneLayout>();
        environment.layer = environmentLayer;
        TrySetTag(environment, "Environment");

        CreateBlock(
            "Floor", environment.transform,
            new Vector3(0f, -0.25f, 0f), new Vector3(90f, 0.5f, 70f),
            floorMaterial, environmentLayer);

        // Outer boundary: 90 x 70 m.  The 1 m thick walls are easy for LiDAR
        // to observe while retaining ample free space for the 3.8 m planner footprint.
        CreateWall("Boundary_North", environment.transform, 0f, 35f, 90f, 1f, wallMaterial, environmentLayer);
        CreateWall("Boundary_South", environment.transform, 0f, -35f, 90f, 1f, wallMaterial, environmentLayer);
        CreateWall("Boundary_West", environment.transform, -45f, 0f, 1f, 70f, wallMaterial, environmentLayer);
        CreateWall("Boundary_East", environment.transform, 45f, 0f, 1f, 70f, wallMaterial, environmentLayer);

        // simple_v3_open_routes: 44 m central opening (z=-14 to z=30).
        // The upper L-wall retains a corner but stops at z=22, leaving a
        // 12.5 m north exit instead of a pocket attached to the boundary.
        CreateWall("Divider_West", environment.transform, -8f, -24.5f, 1f, 21f, wallMaterial, environmentLayer);
        CreateWall("Divider_East", environment.transform, -8f, 32.5f, 1f, 5f, wallMaterial, environmentLayer);
        CreateWall("Upper_Room_South", environment.transform, 14f, 10f, 16f, 1f, wallMaterial, environmentLayer);
        CreateWall("Upper_Room_East", environment.transform, 22f, 16f, 1f, 12f, wallMaterial, environmentLayer);

        // Sparse obstacles test perception without turning the scene into a maze.
        CreateCylinder("Obstacle_A", environment.transform, new Vector3(12f, 1.5f, -18f), 3f, 3f, obstacleMaterial, environmentLayer);
        CreateCylinder("Obstacle_B", environment.transform, new Vector3(28f, 1.5f, -7f), 3f, 3f, obstacleMaterial, environmentLayer);

        RepositionActors(scene);
        RepositionOverviewCamera(scene, new Vector3(0f, 72f, -58f));

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene, OutputScenePath);
        Selection.activeGameObject = environment;
        Debug.Log("Created simplified fire-rescue scene: " + OutputScenePath);
        EditorUtility.DisplayDialog(
            "Fire Rescue",
            "Created and opened FireRescue_Simplified.unity.\n\n" +
            "Robot: (-32, -25)\nVictim 1: (-28, 20)\n" +
            "Victim 2: (32, 22)\nFire: (15, -15)",
            "OK");
    }

    private static void RemoveOldEnvironment(Scene scene)
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            bool oldEnvironment =
                string.Equals(root.name, "FIreScene", StringComparison.OrdinalIgnoreCase) ||
                string.Equals(root.name, "FireScene", StringComparison.OrdinalIgnoreCase) ||
                string.Equals(root.name, "SimplifiedFireScene", StringComparison.OrdinalIgnoreCase) ||
                string.Equals(root.name, "IntermediateFireScene", StringComparison.OrdinalIgnoreCase) ||
                string.Equals(root.name, "ComplexFireScene", StringComparison.OrdinalIgnoreCase) ||
                string.Equals(root.name, "Plane", StringComparison.OrdinalIgnoreCase) ||
                string.Equals(root.name, "Roof", StringComparison.OrdinalIgnoreCase);

            if (oldEnvironment)
            {
                UnityEngine.Object.DestroyImmediate(root);
            }
        }
    }

    private static void RepositionActors(Scene scene)
    {
        GameObject robot = FindRoot(scene, "TrackedRobot");
        if (robot == null)
        {
            throw new InvalidOperationException("TrackedRobot was not found in the source scene.");
        }

        SetActorTransform(robot, new Vector3(-32f, 0f, -25f), Quaternion.identity);
        Rigidbody body = robot.GetComponent<Rigidbody>();
        if (body != null)
        {
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }
        CmdVelSubscriber controller = robot.GetComponent<CmdVelSubscriber>();
        if (controller != null)
        {
            controller.linearAcceleration = 2.5f;
            controller.linearDeceleration = 6f;
            controller.wallContactGraceTime = 0.35f;
            controller.enableContactRecovery = false;
        }

        GameObject victim1 = FindRoot(scene, "Victim_01");
        GameObject victim2 = FindRoot(scene, "Victim_02");
        GameObject fire = FindRoot(scene, "FireSource_01");

        if (victim1 == null || victim2 == null || fire == null)
        {
            throw new InvalidOperationException(
                "Victim_01, Victim_02, or FireSource_01 is missing.");
        }

        SetActorTransform(victim1, new Vector3(-28f, 0f, 20f), Quaternion.identity);
        // Place the second victim behind the divider but align the acoustic
        // bearing through the centre of the wide doorway instead of its corner.
        SetActorTransform(victim2, new Vector3(32f, 0f, 0f), Quaternion.identity);
        FitVictimToGround(victim1);
        FitVictimToGround(victim2);
        SetActorTransform(fire, new Vector3(15f, 0f, -15f), Quaternion.identity);
    }

    private static void RepositionIntermediateActors(Scene scene)
    {
        GameObject robot = FindRoot(scene, "TrackedRobot");
        GameObject victim1 = FindRoot(scene, "Victim_01");
        GameObject victim2 = FindRoot(scene, "Victim_02");
        GameObject fire1 = FindRoot(scene, "FireSource_01");

        if (robot == null || victim1 == null || victim2 == null || fire1 == null)
        {
            throw new InvalidOperationException(
                "TrackedRobot, victim, or fire source is missing from SampleScene.");
        }

        SetActorTransform(robot, new Vector3(-42f, 0f, -32f), Quaternion.identity);
        Rigidbody body = robot.GetComponent<Rigidbody>();
        if (body != null)
        {
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }

        CmdVelSubscriber controller = robot.GetComponent<CmdVelSubscriber>();
        if (controller != null)
        {
            controller.linearAcceleration = 2.5f;
            controller.linearDeceleration = 6f;
            controller.wallContactGraceTime = 0.35f;
            controller.enableContactRecovery = false;
        }

        SetActorTransform(victim1, new Vector3(-40f, 0f, 26f), Quaternion.identity);
        SetActorTransform(victim2, new Vector3(38f, 0f, 0f), Quaternion.identity);
        FitVictimToGround(victim1);
        FitVictimToGround(victim2);

        GameObject victim3 = UnityEngine.Object.Instantiate(victim2);
        victim3.name = "Victim_03";
        SceneManager.MoveGameObjectToScene(victim3, scene);
        SetActorTransform(victim3, new Vector3(38f, 0f, -32f), Quaternion.identity);

        SetActorTransform(fire1, new Vector3(0f, 0f, 25f), Quaternion.identity);
        GameObject fire2 = UnityEngine.Object.Instantiate(fire1);
        fire2.name = "FireSource_02";
        SceneManager.MoveGameObjectToScene(fire2, scene);
        SetActorTransform(fire2, new Vector3(25f, 0f, -25f), Quaternion.identity);
    }

    private static void RepositionComplexActors(Scene scene)
    {
        GameObject robot = FindRoot(scene, "TrackedRobot");
        GameObject victim1 = FindRoot(scene, "Victim_01");
        GameObject victim2 = FindRoot(scene, "Victim_02");
        GameObject fire1 = FindRoot(scene, "FireSource_01");
        if (robot == null || victim1 == null || victim2 == null || fire1 == null)
            throw new InvalidOperationException("TrackedRobot, victim, or fire source is missing from SampleScene.");

        SetActorTransform(robot, new Vector3(-68f, 0f, -55f), Quaternion.identity);
        Rigidbody body = robot.GetComponent<Rigidbody>();
        if (body != null)
        {
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }
        CmdVelSubscriber controller = robot.GetComponent<CmdVelSubscriber>();
        if (controller != null)
        {
            controller.linearAcceleration = 2.5f;
            controller.linearDeceleration = 6f;
            controller.wallContactGraceTime = 0.35f;
            controller.enableContactRecovery = false;
        }

        SetActorTransform(victim1, new Vector3(-66f, 0f, 50f), Quaternion.identity);
        SetActorTransform(victim2, new Vector3(-5f, 0f, 52f), Quaternion.identity);
        FitVictimToGround(victim1);
        FitVictimToGround(victim2);

        GameObject victim3 = UnityEngine.Object.Instantiate(victim2);
        victim3.name = "Victim_03";
        SceneManager.MoveGameObjectToScene(victim3, scene);
        SetActorTransform(victim3, new Vector3(65f, 0f, 52f), Quaternion.identity);

        GameObject victim4 = UnityEngine.Object.Instantiate(victim2);
        victim4.name = "Victim_04";
        SceneManager.MoveGameObjectToScene(victim4, scene);
        SetActorTransform(victim4, new Vector3(67f, 0f, -54f), Quaternion.identity);

        GameObject victim5 = UnityEngine.Object.Instantiate(victim2);
        victim5.name = "Victim_05";
        SceneManager.MoveGameObjectToScene(victim5, scene);
        SetActorTransform(victim5, new Vector3(22f, 0f, -56f), Quaternion.identity);

        SetActorTransform(fire1, new Vector3(-60f, 0f, -5f), Quaternion.identity);
        GameObject fire2 = UnityEngine.Object.Instantiate(fire1);
        fire2.name = "FireSource_02";
        SceneManager.MoveGameObjectToScene(fire2, scene);
        SetActorTransform(fire2, new Vector3(-8f, 0f, -38f), Quaternion.identity);

        GameObject fire3 = UnityEngine.Object.Instantiate(fire1);
        fire3.name = "FireSource_03";
        SceneManager.MoveGameObjectToScene(fire3, scene);
        SetActorTransform(fire3, new Vector3(52f, 0f, 2f), Quaternion.identity);

        GameObject fire4 = UnityEngine.Object.Instantiate(fire1);
        fire4.name = "FireSource_04";
        SceneManager.MoveGameObjectToScene(fire4, scene);
        SetActorTransform(fire4, new Vector3(35f, 0f, 48f), Quaternion.identity);
    }

    static void FitVictimToGround(GameObject victim)
    {
        if (victim == null) return;
        victim.transform.localPosition = new Vector3(victim.transform.localPosition.x, 0f, victim.transform.localPosition.z);
        victim.transform.localScale = Vector3.one * 0.75f;
        // The source mannequin was authored with its head at y=2.15, which
        // places it on top of the test walls. Compress child offsets so the
        // whole victim remains a ground-level, human-sized target.
        foreach (Transform child in victim.transform)
        {
            Vector3 p = child.localPosition;
            p.y *= 0.55f;
            child.localPosition = p;
        }
    }

    private static void RepositionOverviewCamera(Scene scene, Vector3 position)
    {
        GameObject cameraObject = FindRoot(scene, "Main Camera");
        if (cameraObject == null)
        {
            return;
        }

        cameraObject.transform.SetPositionAndRotation(
            position,
            Quaternion.Euler(48f, 0f, 0f));
    }

    private static GameObject FindRoot(Scene scene, string objectName)
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            if (root.name == objectName)
            {
                return root;
            }
        }
        return null;
    }

    private static void SetActorTransform(GameObject actor, Vector3 position, Quaternion rotation)
    {
        Undo.RecordObject(actor.transform, "Reposition " + actor.name);
        actor.transform.SetPositionAndRotation(position, rotation);
        EditorUtility.SetDirty(actor.transform);
    }

    private static void CreateWall(
        string name, Transform parent, float x, float z, float sizeX, float sizeZ,
        Material material, int layer)
    {
        CreateBlock(
            name, parent,
            new Vector3(x, 2.5f, z), new Vector3(sizeX, 5f, sizeZ),
            material, layer);
    }

    private static GameObject CreateBlock(
        string name, Transform parent, Vector3 position, Vector3 scale,
        Material material, int layer)
    {
        GameObject block = GameObject.CreatePrimitive(PrimitiveType.Cube);
        block.name = name;
        block.transform.SetParent(parent, false);
        block.transform.localPosition = position;
        block.transform.localScale = scale;
        SetLayerRecursively(block, layer);
        block.GetComponent<Renderer>().sharedMaterial = material;
        return block;
    }

    private static GameObject CreateCylinder(
        string name, Transform parent, Vector3 position, float diameter, float height,
        Material material, int layer)
    {
        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        obstacle.name = name;
        obstacle.transform.SetParent(parent, false);
        obstacle.transform.localPosition = position;
        obstacle.transform.localScale = new Vector3(diameter, height * 0.5f, diameter);
        SetLayerRecursively(obstacle, layer);
        obstacle.GetComponent<Renderer>().sharedMaterial = material;
        return obstacle;
    }

    private static Material GetOrCreateMaterial(string name, Color color)
    {
        if (!AssetDatabase.IsValidFolder(MaterialFolder))
        {
            Directory.CreateDirectory(MaterialFolder);
            AssetDatabase.Refresh();
        }

        string path = MaterialFolder + "/" + name + ".mat";
        Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (material == null)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
            {
                shader = Shader.Find("Standard");
            }
            material = new Material(shader) { name = name };
            AssetDatabase.CreateAsset(material, path);
        }

        material.color = color;
        EditorUtility.SetDirty(material);
        return material;
    }

    private static void SetLayerRecursively(GameObject target, int layer)
    {
        target.layer = layer;
        foreach (Transform child in target.transform)
        {
            SetLayerRecursively(child.gameObject, layer);
        }
    }

    private static void TrySetTag(GameObject target, string tag)
    {
        try
        {
            target.tag = tag;
        }
        catch (UnityException)
        {
            Debug.LogWarning(tag + " tag does not exist; leaving object untagged.");
        }
    }
}
