# Operation

These controls are documented from the current source code and have not been re-tested in this staged copy.

## Autonomous operation

The main scenes default to ROS velocity control. Once external dependencies and communication are configured, the mission layer selects exploration or victim-guidance goals and FAR supplies routes. Unity executes motion and confirms rescue through trigger occupancy and hold conditions.

## Manual driving

With the Game view receiving keyboard input, `M` switches the `RobotMovement` component between manual and ROS execution. WASD or arrow keys drive and turn; Space clears movement input. `P` saves a demonstration and `R` resets its recording, not the entire mission.

Manual mode disables `CmdVelSubscriber` while it owns robot motion. This is execution-level switching; it does not establish ROS mission pause/resume or shared-control safety. The ROS mission may continue running, so controlled testing is required before demonstrating a handover.

CSV output goes to `Application.persistentDataPath/ManualDemonstrations`. It contains timestamps, Unity/ROS position, yaw, measured velocities, control inputs and contact information. It does not itself save camera frames or point clouds. Switching out of manual mode or stopping may save additional samples automatically.

## Scene configuration

Use the Unity editor and the supplied scene-building scripts. Supported scene generation relies on particular object names and tags, and victim totals are initialised at episode start. Do not assume that arbitrary runtime insertion creates a fully registered rescue target. Fire objects are simulation hazards, not a validated thermal-physics model.
