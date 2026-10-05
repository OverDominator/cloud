// Offline managed-math regression against the project's actual ROSGeometry DLL.
// This does not simulate Unity physics or replace an end-to-end driving test.
using System;
using System.IO;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public static class OdomCoordinateContractTests
{
    static void Near(double actual, double expected, string label)
    {
        if (Math.Abs(actual - expected) > 1e-5)
            throw new Exception(label + ": " + actual + " != " + expected);
    }

    public static int Main(string[] args)
    {
        // Guard the production call site as well as the library's math.
        string source = File.ReadAllText(args[0]);
        if (!source.Contains("angular = Vector3<FLU>.FromUnityAngularVelocity(localAngularVelocity)"))
            throw new Exception("Odom must use axial-vector conversion");
        if (!source.Contains("sampleTime - previousSampleTime") || source.Contains("float elapsed = timer;"))
            throw new Exception("Odom velocity must use actual pose-sample time");

        foreach (float command in new[] { -0.8f, -0.1f, 0f, 0.1f, 0.8f })
        {
            // CmdVelSubscriber negates ROS yaw rate before applying Unity yaw.
            float unityYawRate = -command;
            var reported = Vector3<FLU>.FromUnityAngularVelocity(new Vector3(0, unityYawRate, 0));
            Near(reported.z, command, "command / odom angular sign");
            double dt = 0.1;
            double halfAngle = unityYawRate * dt / 2;
            var pose = FLU.ConvertFromRUF(new Quaternion(0, (float)Math.Sin(halfAngle), 0, (float)Math.Cos(halfAngle)));
            double rosYaw = Math.Atan2(2 * (pose.w * pose.z + pose.x * pose.y),
                1 - 2 * (pose.y * pose.y + pose.z * pose.z));
            Near(reported.z, rosYaw / dt, "pose derivative / odom angular sign");
        }

        var linear = FLU.ConvertFromRUF(new Vector3(0, 0, 1));
        Near(linear.x, 1, "forward linear velocity unchanged");

        // Irregular frame schedule: the limiter's remainder must not inflate
        // the next velocity denominator. Constant 1 m/s must remain 1 m/s.
        double now = 0, previousTime = 0, previousPosition = 0, timer = 0;
        int samples = 0;
        foreach (double frame in new[] { .03, .03, .03, .03, .016, .044, .08, .02 })
        {
            now += frame;
            timer += frame;
            if (timer < .05) continue;
            timer %= .05;
            double elapsed = now - previousTime;
            double position = now;
            Near((position - previousPosition) / elapsed, 1, "irregular frame velocity");
            previousTime = now;
            previousPosition = position;
            samples++;
        }
        Console.WriteLine("PASS: 5 signed yaw cases, forward conversion, " + samples + " irregular-time samples; production contract checks.");
        return 0;
    }
}
