#!/bin/bash

session=particle_filter

tmux kill-session -t $session

tmux new-session -d -s $session

tmux split-window -v 
tmux select-pane -t 0
tmux split-window -h 
tmux select-pane -t 2
tmux split-window -h 

tmux select-window -t 0
tmux select-pane -t 0
tmux send-keys 'roscore' C-m

tmux select-window -t 0
tmux select-pane -t 1
tmux send-keys 'rosrun rviz rviz -d particle-filter.rviz' C-m

sleep 2 

tmux select-window -t 0
tmux select-pane -t 2
tmux send-keys './bin/particle-filter --input cobot3-nsh4.bag' C-m

tmux attach -t $session
