# Fire Navigation Experiment Plan

## Baseline navigation scenarios

Run each scenario at least five times from the same initial pose.

| Scenario | Purpose | Suggested goal |
| --- | --- | --- |
| B1 Open area | Basic reachability and stopping | Straight, medium distance |
| B2 Corridor turn | Global/local path tracking | Goal after one corner |
| B3 Near-wall route | Clearance and collision behavior | Reachable goal near a wall |
| B4 Multi-room | Long-range planning | Goal in another room |

## Metrics

- Navigation success rate
- Travel time
- Travelled path length
- FAR mean and P95 planning time
- Collision count
- Maximum commanded linear/angular speed
- ROS state-estimation message rate

## Required screenshots per scenario

1. Unity overview showing robot start and goal area.
2. RViz after the global path appears.
3. RViz near the middle of navigation with point cloud and paths visible.
4. Unity at the final robot pose.
5. One representative failure screenshot, if a run fails.

Use the same RViz camera angle and Unity scene camera for repeated comparisons.

## Recording commands

From WSL:

```bash
cd "/mnt/c/Users/34439/Saved Games/Unity/Project/FireScene"
bash Tools/Experiments/record_navigation_experiment.sh B1_open_area
```

Stop recording with `Ctrl+C` only after the robot stops. Point clouds are omitted by default to keep bags small. Add `--with-cloud` only for a representative replay:

```bash
bash Tools/Experiments/record_navigation_experiment.sh B1_open_area --with-cloud
```

Analyze a completed run:

```bash
python3 Tools/Experiments/analyze_navigation_bag.py ExperimentData/B1_open_area/RUN_ID/navigation.bag
```
