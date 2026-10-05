# Setup status

## Known environment

- Unity editor: `6000.0.75f1`, from the source project's `ProjectVersion.txt`.
- Windows-hosted Unity and WSL distribution `Ubuntu-20.04`.
- ROS Noetic, as referenced by the supplied launch helpers.
- Embedded ROS-TCP-Connector package metadata: `0.7.0-preview`. Local modifications are retained.
- Depth branch requirements are listed in `ros/fire_perception/requirements-depth.txt`; exact working package versions still need verification.

## Unity project

Open `unity/` in the matching editor. Core scenes include `SampleScene`, `FireRescue_Simplified` (Original configuration) and `FireRescue_Easy`. The existing build settings initially select `SampleScene`; open the required scene explicitly. Preserve `.meta` files.

Do not start the staged copy alongside the original connected simulation: both can use the same ROS endpoint and topics. A clean import and isolated run have not yet been performed.

## Missing external components

The original runtime additionally uses FAR Planner, ROS-TCP-Endpoint and the autonomous exploration development environment, including local-planning and terrain-processing packages. They are not bundled here. Their exact commits, uncommitted changes and licence conditions must be captured before a reproducible install procedure is promised.

The legacy `Tools/ROS/start_fire_nav.sh` assumes `/root/catkin_ws` and `/root/autonomous_exploration_development_environment`. `install_shortcuts.sh` also writes to a user-specific location. These files are retained for inspection, not presented as portable one-command installers. After separating the Unity project into `unity/`, helpers which infer the old project root also require review.

The depth localiser defaults to `depth-anything/Depth-Anything-V2-Small-hf`, with configuration overrides. Model weights are not included. Verify the active model configuration and its licence before writing download instructions.

## Pending validation

Complete dependency capture, configurable paths and endpoint settings first. Then test in a separate workspace: import Unity, build ROS packages, establish one communication endpoint, start one planning/mission stack, and run one isolated rescue demonstration. No fresh-install success is claimed for this release candidate.
