#!/bin/sh
xterm -e "source ../../devel/setup.sh; 
					export TURTLEBOT_GAZEBO_WORLD_FILE=/home/user/Desktop/home-service-robot/catkin_ws/src/map/navarrs.world
          roslaunch turtlebot_gazebo turtlebot_world.launch" & 
sleep 5

xterm  -e  "source ../../devel/setup.sh; 
						export TURTLEBOT_GAZEBO_MAP_FILE=/home/user/Desktop/home-service-robot/catkin_ws/src/map/map.yaml
            roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

xterm  -e  "source ../../devel/setup.sh; 
            roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

xterm  -e  "source ../../devel/setup.sh; 
            rosrun add_markers add_markers" &