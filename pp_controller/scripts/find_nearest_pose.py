#!/usr/bin/env python

import rospy
import sys
import os
import math
import csv

from nav_msgs.msg import Odometry
from std_msgs.msg import Int64

# car_name        = str(sys.argv[1])
# trajectory_name = str(sys.argv[2])
global it
global old_index

it = 1
old_index = -1
plan = []

# CHANGE THE NAME OF THE TOPICS
min_index_pub = rospy.Publisher('/ego_vehicle/PP_control/index_nearest_point', Int64, queue_size = 1)
ind_last_point_pub  = rospy.Publisher('/ego_vehicle/PP_control/index_last_point', Int64, queue_size = 1)

def construct_path():
    file_path = os.path.expanduser('/home/swapy/catkin_ws/src/pp_controller/path/odom_final.csv')

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    last_point_index = plan.index(plan[-1])
    ind_last_point_pub.publish(last_point_index)
    print('max_index',last_point_index)

def odom_callback(data):
    global it
    global old_index
    min_index      = Int64()
    curr_x         = data.pose.pose.position.x
    curr_y         = data.pose.pose.position.y
    index          = find_nearest_point(curr_x, curr_y)
    if index == None:
	min_index.data = old_index
    else:
	min_index.data = index
    if it == 1:
	min_index_pub.publish(min_index)
	print('min_index',min_index)
    	old_index = index
	it = 0
    if index != old_index:
	min_index.data = index
	min_index_pub.publish(min_index)
	print('min_index',min_index)

    old_index = index	    

def find_nearest_point(curr_x, curr_y):
    ranges = []
    for index in range(0, len(plan)):
        eucl_x = math.pow(curr_x - plan[index][0], 2)
        eucl_y = math.pow(curr_y - plan[index][1], 2)
        eucl_d = math.sqrt(eucl_x + eucl_y)
        ranges.append(eucl_d)
    return(ranges.index(min(ranges)))

if __name__ == '__main__':
    try:
        rospy.init_node('nearest_pose_isolator', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
        rospy.Subscriber('/gps/odom', Odometry, odom_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
