# Setup status

[简体中文](zh-CN/setup.md)

## Known environment

- Unity editor: `6000.0.75f1`, from the source project's `ProjectVersion.txt`.
- Windows-hosted Unity and WSL distribution `Ubuntu-20.04`.
- ROS Noetic, as referenced by the supplied launch helpers.
- Embedded ROS-TCP-Connector package metadata: `0.7.0-preview`. Local modifications are retained.
- Depth branch requirements are listed in `ros/fire_perception/requirements-depth.txt`; exact working package versions still need verification.

## Unity project

Open `unity/` in the matching editor. Core scenes include `SampleScene`, `FireRescue_Simplified` (Original configuration) and `FireRescue_Easy`. The existing build settings initially select `SampleScene`; open the required scene explicitly. Preserve `.meta` files.

Do not start the staged copy alongside the original connected simulation: both can use the same ROS endpoint and topics. Import, isolated builds, communication and a complete autonomous run have been checked on the existing Windows/WSL machine. This is not a fresh-machine installation test.

## Missing external components

The original runtime additionally uses FAR Planner, ROS-TCP-Endpoint and the autonomous exploration development environment, including local-planning and terrain-processing packages. They are not bundled here. Their exact commits, uncommitted changes and licence conditions must be captured before a reproducible install procedure is promised.

The legacy `Tools/ROS/start_fire_nav.sh` assumes `/root/catkin_ws` and `/root/autonomous_exploration_development_environment`. `install_shortcuts.sh` also writes to a user-specific location. These files are retained for inspection, not presented as portable one-command installers. After separating the Unity project into `unity/`, helpers which infer the old project root also require review.

The depth localiser defaults to `depth-anything/Depth-Anything-V2-Small-hf`, with configuration overrides. Model weights are not included. Verify the active model configuration and its licence before writing download instructions.

## Prepared-environment entrypoint and validation scope

Use `Tools/ROS/start_release_nav.sh` with `FIRE_ROS_SETUP` and `FIRE_CMU_SETUP` pointing to already-built workspaces and `FIRE_RUN_DIR` to a new run directory. The workspaces must provide the modified Unity integration launch file. `--check` checks package and launch-file discovery; it does not build dependencies or start the system. Without `--check`, the launcher starts the prepared ROS stack; Unity and visualization windows are started separately.

The isolated validation and the later recorded demonstration are different runs. The demonstration used this entrypoint. See [validation evidence](full_validation_complete.txt), [demonstration evidence](recorded_demo.txt) and [entrypoint notes](portable_entrypoint.txt). Missing depth weights allow the localizer to continue with LiDAR-only localization; confirm model readiness for depth-enabled testing.

External dependency delivery, exact version and patch capture, endpoint documentation and fresh-machine installation remain pending. No fresh-install success or exact reproduction of the historical 200-trial configuration is claimed.
