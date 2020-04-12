#!/bin/zsh
xterm -e "source ../../devel/setup.zsh; 
					export TURTLEBOT_GAZEBO_WORLD_FILE=/home/navars/Desktop/nanodegree/module5_pathplanning/home-service-robot/catkin_ws/src/map/navarrs.world
          roslaunch turtlebot_gazebo turtlebot_world.launch" & 
sleep 5

xterm  -e  "source ../../devel/setup.zsh; 
						export TURTLEBOT_GAZEBO_MAP_FILE=/home/navars/Desktop/nanodegree/module5_pathplanning/home-service-robot/catkin_ws/src/map/map.yaml
            roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

xterm  -e  "source ../../devel/setup.zsh; 
            roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

xterm  -e  "source ../../devel/setup.zsh; 
            rosrun pick_objects pick_objects" &

sleep 2

xterm  -e  "source ../../devel/setup.zsh; 
            rostopic echo /location_home_service" &