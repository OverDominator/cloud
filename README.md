# Unity ROS Fire Rescue

A configurable Unity–ROS software-in-the-loop prototype for indoor robot search and rescue. The autonomous workflow connects exploration, simulated victim cues, FAR-based navigation, motion execution and rescue confirmation. The current implementation also includes keyboard driving and manual trajectory recording.

This is a **private release candidate**, not a verified standalone distribution. It is hosted at [OverDominator/cloud](https://github.com/OverDominator/cloud). External ROS dependencies, portability and redistribution checks remain open; see [release checklist](docs/release_checklist.md). The local dependency snapshot and full verification evidence directories are not included in this repository.

## Demonstration video

[Download the full Easy scene demonstration](media/fire-rescue-demo-v0.1.0.mp4). Repository access is required while this repository remains private.

The video shows the Unity observer overview, the robot RGB image and the ROS/RViz planning view from the same run. It preserves the full recording at original speed, with explanatory labels and an end-of-video result caption; no narration or audio track is included. The observer view is not robot perception, and the RGB image is not physical thermal imaging.

All four simulated victims were rescued. The task logger recorded 284.684 seconds and 179.979 metres of travel, with zero logged collisions. This is a separate demonstration, not an additional trial in the main 200 results. See [demonstration evidence](docs/recorded_demo.txt) and [trial summary](results/demo_20261005/trials.csv). It demonstrates one successful run, not guaranteed success or fresh-machine reproducibility.

## Validation status

On 5 October 2026, an isolated verification copy of the Easy scene completed one full autonomous run: all four simulated victims were rescued in approximately 205.6 seconds, with approximately 131.6 metres of recorded travel. This run used independently built local ROS dependencies and separate logs. It is a release check, not an additional trial in the main 200 results. See [validation evidence summary](docs/full_validation_complete.txt).

Unity 6000.0.75f1 import, ROS Noetic builds, launch resolution, two-way communication and offline Depth Anything V2 loading have been checked on the existing Windows/WSL machine. Installation on a fresh machine, repeatability across runs, and exact historical dependency identity remain unverified.

The current command adapter excludes active wall-following, reverse-escape and trace-back recovery routines. Safety checks and bounded negative commands from the planner remain. Supplementary test scenes and their executable helpers have been removed from this candidate; supplementary result CSVs remain historical data only.

## Starting a prepared environment

After building the release package and the modified external dependencies, specify their workspace setup files and check package discovery:

```bash
FIRE_ROS_SETUP=/path/to/ros_workspace/devel/setup.bash \
FIRE_CMU_SETUP=/path/to/cmu_workspace/devel/setup.bash \
FIRE_RUN_DIR=/path/to/new_run_directory \
bash Tools/ROS/start_release_nav.sh --check
```

The supplied workspaces must expose FAR, the custom Unity integration and ROS-TCP-Endpoint. This check does not build or download them. The recorded demonstration used this launcher in the prepared Windows/WSL validation environment, with Unity and visualization windows started separately. This does not establish fresh-machine installation or exact historical reproduction. See [entrypoint notes](docs/portable_entrypoint.txt). Older shortcut scripts remain machine-specific.

Depth Anything V2 requires separately prepared model files. Missing weights cause the existing localizer to continue with LiDAR-only localization; confirm the model-ready status when testing the intended depth-enabled pipeline. See [model dependency notes](docs/perception_dependency_check.txt).

## Included components

- `unity/`: Unity project assets, scene configuration and the locally used ROS-TCP-Connector package.
- `ros/fire_perception/`: perception, mission management, safety and recording code for the FAR workflow.
- `Tools/`: existing inspection and recording helpers. Their machine-specific paths are not yet portable.
- `results/main_200/`: four source batch summaries and selected archived code snapshots.
- `results/supplementary_30/`: selected local-obstacle outcomes, reported separately.
- `third_party/`: preserved connector licence and acknowledgements.

Start with [setup status](docs/setup.md), [operation](docs/operation.md) and [experimental scope](docs/experiments.md).

## Results and scope

The Original scene completed 61 of 100 valid terminal trials and the Easy scene completed 92 of 100, with a 600 s limit per trial. The selected supplementary batches contain ten completed single-victim trials per configuration: End Wall, Right Angle and Left Angle. These are different experiments and are not pooled.

These results evaluate autonomous operation under specified simulation assumptions, not manual-driving effectiveness or professional firefighter training. The current development code is not labelled as an exact copy of the 200-trial configuration. Archived snapshots are partial evidence, not a complete executable reproduction bundle.

Robot pose and fire locations use simulation information. Visual target cues are simplified. The project does not establish physical radar, thermal-radiation sensing, independent SLAM accuracy or real-fire deployment performance. FAR Planner and Depth Anything V2 are existing methods, not algorithms introduced by this project.

## Licence status

No project-wide licence has been assigned to this release candidate. The ROS package declares MIT in its metadata, but ownership and redistribution scope must be reviewed before publication. Third-party rights remain with their respective owners. See [third-party notices](THIRD_PARTY_NOTICES.md).
