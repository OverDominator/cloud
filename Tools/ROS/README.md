# Fire navigation quick start

From WSL, run:

```bash
~/start_fire_nav.sh
```

Then wait for the ROS TCP Endpoint to listen on port `10001` and start Unity Play.

`system_unity.launch` starts `stateEstimationAligner`, which converts `/odom` to
the Z-aligned `/state_estimation`. Do not run a separate topic relay for these
topics, because multiple publishers would feed conflicting poses to the planners.

Inspect the managed ROS terminals:

```bash
tmux attach -t fire_nav
```

Switch windows with `Ctrl+B`, then `N` or `P`. Detach without stopping ROS with
`Ctrl+B`, then `D`.

Stop all processes started by the helper:

```bash
~/stop_fire_nav.sh
```

The stop helper does not terminate ROS processes that were started manually in
other terminals.
