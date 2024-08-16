#!/bin/sh

# Define workspace path
path_to_ws="/home/segk/home_service_robot"

gnome-terminal -- bash -c "cd ${path_to_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo turtlebot_world.launch; exec bash" &
sleep 20
gnome-terminal -- bash -c "cd ${path_to_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo amcl_demo.launch; exec bash" & 
sleep 15
gnome-terminal -- bash -c "cd ${path_to_ws} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch; exec bash"

