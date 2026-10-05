# Experimental scope

## Main evaluation

The three Original-scene source batches contain 10, 10 and 80 trials. Their aggregate is 100 trials, 61 completed and 39 timed out. Easy contains 100 trials, 92 completed and 8 timed out. Keep the source batch files for provenance; report Original as one 100-trial scene group.

The denominator contains valid terminal records. Startup failures without such records and manual interruptions are outside it. The limit is 600 s per main trial. Active reversing recovery was disabled in the main evaluated configuration. Current manual-driving functions are not evaluated by these summaries.

`source_snapshots/` preserves selected archived Python and C# files exactly. These do not include every dependency, parameter or historical scene needed to recreate the full environment. They are not drop-in replacements for the current code. Connector snapshots retain third-party status.

## Supplementary evaluation

`right_left_angle.csv` selects RightAngle and LeftAngle from `Formal30_20261004_120550/summary.csv`. `end_wall_selected_rerun.csv` uses `WallEndShort10_Rerun10_20261004_161704/summary.csv`. Only scenario, attempt, yaw, status and elapsed time are retained; the source records remain untouched.

There are ten selected successful single-victim trials per configuration. End Wall was simplified and retested after earlier outcomes; these selected batches do not represent all attempted runs. Their timing origin differs from the main mission logger, so absolute durations are not directly comparable and the 30 trials must not be pooled with the 200 main trials.

Earlier End Wall outcomes and excluded attempts remain in the original local archive and should be explained in the full experimental documentation before public release. No raw images, complete sensor bags or claim of frame-by-frame replay is included here.
