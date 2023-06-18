#ifndef MUEAVI_TF_ROS_HPP
#define MUEAVI_TF_ROS_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>

#include "fast-cpp-csv-parser/csv.h"

const std::vector<geometry_msgs::TransformStamped> csv_to_tf_msg(void);

#endif