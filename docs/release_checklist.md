# Release checklist

[简体中文](zh-CN/release_checklist.md)

## Completed staging work

- Created a separate local candidate without changing the running project.
- Preserved core scenes, scripts, Unity metadata and the active connector package.
- Excluded generated pilot scenes, paper-production scripts, caches, personal dissertation files and runtime logs.
- Excluded alternative-planner files outside the FAR release scope; removed their two Python install entries from the staged CMake file only.
- Preserved four main-batch summaries and partial archived source snapshots.
- Exported the selected 30 supplementary outcomes with explicit selection limitations.
- Checked Unity import, isolated ROS builds, communication and one complete autonomous task on the existing machine.
- Recorded a separate complete demonstration and uploaded the candidate to the private repository `OverDominator/cloud`.

`copy_manifest.csv` records source-relative paths and hashes at copy time. The staged CMake file was subsequently edited as described above; this manifest is not a final-package hash list.

## Remaining distribution and reproducibility work

- Verify source ownership, asset provenance and project-wide licensing with the owner.
- Capture external ROS dependency commits and local patches, and reconcile WSL runtime code with the Windows copy.
- Replace user-specific paths and document endpoint configuration; do not run legacy installation helpers blindly.
- Verify installation on a fresh machine; the existing-machine isolated run does not replace this test.
- Demonstrate manual control, CSV output and safe return to ROS; do not infer mission pause/resume.
- Retain the recorded demonstration and distinguish its results from the historical main and supplementary experiments.
- Review files for credentials, private endpoints, usernames and paths. Automated pattern checks are only a preliminary screen.
- Public visibility was authorized and enabled on 6 October 2026. See [publication review](publication_review_20261006.md). Project-wide licensing remains unresolved.
- Keep publication status consistent across languages; do not equate public access with a blanket reuse licence or a complete reproducible distribution.
