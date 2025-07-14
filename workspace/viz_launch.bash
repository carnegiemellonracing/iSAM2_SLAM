#!/bin/bash

NODE="controls_sim_slam_node"
SESSION_NAME="VIZ_LAUNCH"

tmux new -d -s $SESSION_NAME

tmux split-window -h -t $SESSION_NAME
tmux split-window -h -t $SESSION_NAME:0.1
tmux select-pane -t $SESSION_NAME:0.0 -T "Node"
tmux select-pane -t $SESSION_NAME:0.1 -T "Rosbag"
tmux select-pane -t $SESSION_NAME:0.2 -T "Visualizer"

tmux send-keys -t $SESSION_NAME:0.0 'source install/setup.bash' C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run isam2 $NODE" C-m

tmux send-keys -t $SESSION_NAME:0.1 'source install/setup.bash' C-m
tmux send-keys -t $SESSION_NAME:0.1 'cd ~/rosbags' C-m
tmux send-keys -t $SESSION_NAME:0.1 'ros2 bag play controls_sim_bag' C-m

tmux send-keys -t $SESSION_NAME:0.2 'cd viz' C-m
tmux send-keys -t $SESSION_NAME:0.2 'python3 visualize.py' C-m

tmux attach-session -t $SESSION_NAME