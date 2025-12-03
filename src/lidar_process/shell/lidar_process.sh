#!/bin/bash

# 创建会话和第一个窗口
tmux new-session -d -s ros_session -n main_nodes

# Pane 0:TW360.launch
tmux send-keys -t ros_session:0 'roslaunch tanwaylidar_view TW360.launch' C-m

# Pane 1:sector_min_filter.launch
tmux split-window -h -t ros_session:0
tmux send-keys -t ros_session:0.1 'sleep 5; roslaunch lidar_process sector_min_filter.launch' C-m

# 整理第一个窗口布局
tmux select-layout -t ros_session:0 tiled
tmux attach-session -t ros_session:0
