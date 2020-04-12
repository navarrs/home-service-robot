#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/navars/Desktop/nanodegree/module5_pathplanning/catkin_ws/src/map/navarrs.world

xterm -e "source ../../devel/setup.zsh;
 					roslaunch turtlebot_gazebo turtlebot_world.launch" & 
sleep 5

xterm  -e  "source ../../devel/setup.zsh; 
            roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

xterm  -e  "source ../../devel/setup.zsh; 
            roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm  -e  "source ../../devel/setup.zsh; 
            roslaunch turtlebot_teleop keyboard_teleop.launch" &