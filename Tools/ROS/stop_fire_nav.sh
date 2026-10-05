#!/usr/bin/env bash
set -euo pipefail

SESSION="fire_nav"

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "No managed fire navigation session is running."
    exit 0
fi

while IFS= read -r window; do
    tmux send-keys -t "$SESSION:$window" C-c
done < <(tmux list-windows -t "$SESSION" -F '#{window_name}')

sleep 3
tmux kill-session -t "$SESSION" 2>/dev/null || true
echo "Managed fire navigation processes stopped."
