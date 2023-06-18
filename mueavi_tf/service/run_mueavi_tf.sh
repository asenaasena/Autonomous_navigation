#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.57.1:11311
export ROS_HOSTNAME=$(hostname).local
source ~/catkin_ws/devel/setup.bash
export ROS_NAMESPACE=core01
roslaunch mueavi_tf mueavi_tf.launch
