#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for name in start_fire_nav.sh stop_fire_nav.sh; do
    target="/root/$name"
    if [[ -e "$target" && ! -L "$target" ]]; then
        cp "$target" "$target.before_fire_perception.bak"
    fi
    ln -sfn "$SCRIPT_DIR/$name" "$target"
done

echo "ROS shortcuts installed."
