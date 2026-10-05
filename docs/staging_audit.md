# Local staging audit

Audit date: 2026-10-05. This is a static packaging check, not a successful build or runtime test.

## Passed checks

- Copied-file SHA256 values match the source-copy manifest except the intentionally edited ROS CMake file.
- Main summaries contain 200 rows: Original batches 10/10/80 with 6/8/47 completions, and Easy 100 with 92 completions.
- Selected supplementary exports contain 10 End Wall and 20 combined Right/Left Angle rows.
- Included Python files parse successfully without executing them.
- Unity package manifest and lockfile parse as JSON.
- Included scene, prefab, material and asset references do not point to omitted assets found in the original local GUID inventory. This does not validate registry packages or perform a Unity import.
- All Python scripts listed in the staged ROS CMake install list exist.
- A preliminary scan found no matching GitHub-token, AWS-access-key-ID or PEM private-key signatures. This is not an exhaustive security audit.

## External dependency findings

- ROS-TCP-Endpoint checkout HEAD: `993d366b8900bf9f3d2da444fde64c0379b4dc7c`; `git status --short` returned no entries at inspection.
- Autonomous exploration environment HEAD: `bf0cba71365271ebff09831a05afd78578150300`. Its tracked diff affects eight files (30 insertions, 29 deletions), including local-planning, terrain and vehicle-simulator configuration/source. HEAD alone is insufficient to reproduce the working environment; untracked content also needs inventory.
- The FAR directory did not resolve as a Git repository, so its upstream revision remains unverified.
- These observations describe the current WSL environment, not the historical 200-trial configuration.

## Remaining portability issues

Twelve files in current Unity/ROS/Tools scope contain machine-path patterns. They include ROS start/install helpers, the navigation recorder, victim configuration, launch configuration, experiment logger and diagnostic helpers. These paths were retained to preserve behaviour and are not necessarily secrets, but must be parameterised or removed before public release. Archived snapshots intentionally preserve historical source and need a separate privacy review.

No Git repository, remote, commit, public upload, clean import, catkin build or Unity/ROS runtime validation was performed. No video or model weights are included.
