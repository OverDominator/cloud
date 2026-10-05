# Release checklist

## Completed staging work

- Created a separate local candidate without changing the running project.
- Preserved core scenes, scripts, Unity metadata and the active connector package.
- Excluded generated pilot scenes, paper-production scripts, caches, personal dissertation files and runtime logs.
- Excluded alternative-planner files outside the FAR release scope; removed their two Python install entries from the staged CMake file only.
- Preserved four main-batch summaries and partial archived source snapshots.
- Exported the selected 30 supplementary outcomes with explicit selection limitations.

`copy_manifest.csv` records source-relative paths and hashes at copy time. The staged CMake file was subsequently edited as described above; this manifest is not a final-package hash list.

## Required before public upload

- Verify source ownership, asset provenance and project-wide licensing with the owner.
- Capture external ROS dependency commits and local patches, and reconcile WSL runtime code with the Windows copy.
- Replace user-specific paths and document endpoint configuration; do not run legacy installation helpers blindly.
- Verify Unity asset references and package import in a fresh location, then build and run the isolated ROS/Unity chain.
- Demonstrate manual control, CSV output and safe return to ROS; do not infer mission pause/resume.
- Prepare a real autonomous demonstration video and representative screenshots.
- Review files for credentials, private endpoints, usernames and paths. Automated pattern checks are only a preliminary screen.
- Decide the GitHub account, repository name, visibility and licence. No repository has been created and nothing has been uploaded.
- Add a verified repository link to the dissertation only after publication is complete.
