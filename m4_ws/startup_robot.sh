#!/bin/sh



# Create a new tmux session named "robot"
tmux new-session -d -s robot

# Split the window into 4 panes in square formation
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2   
tmux split-window -v
tmux select-pane -t 3
tmux split-window -h

# Select pane 3
tmux select-pane -t 3

# cd to m4v2-code/
tmux send-keys -t robot:0.3 'cd ..' C-m

# add delay
sleep 10

# run interfaces
tmux send-keys -t robot:0.3 '/home/x1m4/x1m4/interface.sh' C-m

# Select pane 0
tmux select-pane -t 0

# reset roboclaw
#tmux send-keys -t robot:0.0 './reset_roboclaw.sh' C-m
tmux send-keys -t robot:0.0 'x1m4' C-m

sleep 1

# launch tilt controller
tmux send-keys -t robot:0.0 '/home/x1m4/x1m4/m4_ws/launch_tilt.sh' C-m


# Select pane 4
tmux select-pane -t 4

# launch drive controller
tmux send-keys -t robot:0.4 '/home/x1m4/x1m4/m4_ws/launch_drive.sh' C-m

