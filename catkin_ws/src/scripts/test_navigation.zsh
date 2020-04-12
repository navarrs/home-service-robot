#!/bin/zsh



xterm -e "source ../../devel/setup.zsh; 
					export TURTLEBOT_GAZEBO_WORLD_FILE=/home/navars/Desktop/nanodegree/module5_pathplanning/catkin_ws/src/map/navarrs.world
          roslaunch turtlebot_gazebo turtlebot_world.launch" & 
sleep 5

xterm  -e  "source ../../devel/setup.zsh; 
						export TURTLEBOT_GAZEBO_MAP_FILE=/home/navars/Desktop/nanodegree/module5_pathplanning/catkin_ws/src/map/map.yaml
            roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

xterm  -e  "source ../../devel/setup.zsh; 
            roslaunch turtlebot_rviz_launchers view_navigation.launch" &