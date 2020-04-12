# Robotics Nanodegree - Home Service Robot 

This is my Udacity's Robotics Nanodegree final project ```Home Service Robot```.

### Requirements
For this project, the following packages are required:
- [gmapping](http://wiki.ros.org/gmapping)
- [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
- [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
- [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)

### Build project
Initialize workspace:
```
	cd /path/to/catkin_ws/src/
	catkin_init_workspace
```

To compile:
```
	cd /path/to/catkin_ws/
	catkin_make
```

## Scripts
Note: on the scripts, I set a couple of variables with my custom paths. Make sure these variables match your paths:
- TURTLEBOT_GAZEBO_WORLD_FILE
- TURTLEBOT_GAZEBO_MAP_FILE

### Test SLAM
Deploys a turtlebot inside an environment found in ```catkin_ws/src/map```, and control it with the keyboard to map the environment. 

To run the script:
```
	cd /path/to/catkin_ws/
	./src/scripts/test_slam.zsh
```

Gazebo should look like this:
<p align="center"><img src="./readme/gazebo.png" width="800" /></p>

Rviz should look like this:
<p align="center"><img src="./readme/rviz_slam.png" width="800" /></p> 

### Test navigation 
Tests the ROS navigation stack. Add a goal location on RVIZ using the 2D Nav Goal and test if the robot moves toward it and orients itself correctly. 

To run the script:
```
	cd /path/to/catkin_ws/
	./src/scripts/test_navigation.zsh
```

### Pick Objects

### Add Markers

### Home Service Robot