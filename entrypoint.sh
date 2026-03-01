#!/bin/bash
set -euo pipefail

session_name="boids"
workspace_cmd=("python3" "main.py")

pushd /workspace >/dev/null

if ! tmux has-session -t "$session_name" 2>/dev/null; then
  tmux new-session -d -s "$session_name" "${workspace_cmd[*]}"
else
  tmux send-keys -t "$session_name" C-c
  tmux send-keys -t "$session_name" "${workspace_cmd[*]}" Enter
fi

exec tmux attach-session -t "$session_name"
