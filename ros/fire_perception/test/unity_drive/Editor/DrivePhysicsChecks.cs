using System;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class DrivePhysicsChecks
{
    const float Step = .02f;
    static void Near(float actual, float expected, float tolerance, string label)
    {
        if (Mathf.Abs(actual - expected) > tolerance)
            throw new Exception(label + ": " + actual + " expected " + expected);
    }

    public static void Run()
    {
        try
        {
            // This runner has its own disposable editor process/project.
            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            Physics.simulationMode = SimulationMode.Script;
            PhysicsScene physics = scene.GetPhysicsScene();
            GameObject robot = GameObject.CreatePrimitive(PrimitiveType.Cube);
            SceneManager.MoveGameObjectToScene(robot, scene);
            Rigidbody body = robot.AddComponent<Rigidbody>();
            body.useGravity = false;
            body.linearDamping = body.angularDamping = 0;
            body.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;

            // Reproduce the old zero-command pose-drive as a comparison.
            body.angularVelocity = new Vector3(0, .075f, 0);
            for (int i = 0; i < 100; i++)
            {
                body.MoveRotation(body.rotation);
                physics.Simulate(Step);
            }
            float legacyDrift = Mathf.DeltaAngle(0, body.rotation.eulerAngles.y);
            Debug.Log("DRIVE_LEGACY_COMPARISON: zero-command drift over 2 seconds = " + legacyDrift + " degrees");
            body.rotation = Quaternion.identity;

            // A zero command must remove inherited spin, in actual PhysX.
            body.angularVelocity = new Vector3(0, .075f, 0);
            for (int i = 0; i < 100; i++)
            {
                PlanarDriveExecution.ApplyYaw(body, body.rotation, 0);
                physics.Simulate(Step);
            }
            Near(Mathf.DeltaAngle(0, body.rotation.eulerAngles.y), 0, .001f, "zero command yaw drift");
            Near(body.angularVelocity.y, 0, .0001f, "zero command angular velocity");

            foreach (float rate in new[] { -.8f, -.1f, .1f, .8f })
            {
                body.rotation = Quaternion.identity;
                body.angularVelocity = Vector3.up * 2; // old motion must not add to command
                for (int i = 0; i < 50; i++)
                {
                    Quaternion next = body.rotation * Quaternion.Euler(0, rate * Step * Mathf.Rad2Deg, 0);
                    PlanarDriveExecution.ApplyYaw(body, next, rate);
                    physics.Simulate(Step);
                }
                Near(Mathf.DeltaAngle(0, body.rotation.eulerAngles.y) * Mathf.Deg2Rad, rate, .003f, "signed constant turn");
                PlanarDriveExecution.ApplyYaw(body, body.rotation, 0);
                physics.Simulate(Step);
                Near(body.angularVelocity.y, 0, .0001f, "stop after turn");
            }

            body.position = Vector3.zero;
            body.rotation = Quaternion.identity;
            for (int i = 0; i < 50; i++)
            {
                body.linearVelocity = Vector3.forward * .4f;
                PlanarDriveExecution.ApplyYaw(body, body.rotation, 0);
                physics.Simulate(Step);
            }
            Near(body.position.z, .4f, .002f, "straight line distance");
            Near(body.position.x, 0, .001f, "straight line lateral drift");

            // Direct velocity control must still be resolved against colliders.
            GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
            SceneManager.MoveGameObjectToScene(wall, scene);
            wall.transform.position = new Vector3(0, 0, 2);
            wall.transform.localScale = new Vector3(10, 10, 1);
            Physics.SyncTransforms();
            for (int i = 0; i < 150; i++)
            {
                body.linearVelocity = Vector3.forward;
                PlanarDriveExecution.ApplyYaw(body, body.rotation, 0);
                physics.Simulate(Step);
            }
            if (body.position.z > 1.05f || body.position.z < .9f)
                throw new Exception("Collider blocking failed: z=" + body.position.z);
            Debug.Log("DRIVE_PHYSICS_PASS: residual-spin stop, 4 signed turns and stops, straight motion, collider blocking. Production PlanarDriveExecution used.");
            EditorApplication.Exit(0);
        }
        catch (Exception error)
        {
            Debug.LogException(error);
            EditorApplication.Exit(1);
        }
    }
}
