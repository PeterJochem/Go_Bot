#!/bin/bash
session="go_bot"
catkin_ws_path="/home/peterjochem/Desktop/Go_Bot/catkin_ws/"
catkin_source_path="${catkin_ws_path}devel/setup.bash"

tmux new -s $session -d

tmux rename-window -t 0 'roscore'
tmux send-keys -t "go_bot:roscore" "source ${ros_source_path}" C-m
tmux send-keys -t "go_bot:roscore" "source ${catkin_source_path}" C-m
tmux send-keys -t "go_bot:roscore" "roscore" C-m

sleep 2
tmux new-window -t $session:1 -n 'interobotix'
tmux send-keys -t "go_bot:interobotix" "source ${catkin_source_path}" C-m
tmux send-keys -t "go_bot:interobotix" "roslaunch --wait interbotix_moveit interbotix_moveit.launch robot_name:=rx200 use_gazebo:=true" C-m

tmux new-window -t $session:2 -n 'go_motion_planning'
tmux send-keys -t "go_bot:go_motion_planning" "source ${catkin_source_path}" C-m
tmux send-keys -t "go_bot:go_motion_planning" "roslaunch --wait go_motion_planning launch.launch" C-m

tmux new-window -t $session:3 -n 'manipulate_gazebo'
tmux send-keys -t "go_bot:manipulate_gazebo" "source ${catkin_source_path}" C-m
tmux send-keys -t "go_bot:manipulate_gazebo" "rosrun go_motion_planning manipulate_gazebo.py" C-m

tmux new-window -t $session:4 -n 'game_server'
tmux send-keys -t "go_bot:game_server" "source ${catkin_source_path}" C-m
tmux send-keys -t "go_bot:game_server" "uvicorn main:app --reload" C-m
