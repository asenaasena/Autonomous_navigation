#!/usr/bin/env python

import rospy
import sys
import os
import math
import csv

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64

global plan
plan = []

global plan_size
global seq

global last_point_index
last_point_index = -1

plan         = []
frame_id     = 'map'
seq          = 0

ang_goal_pub = rospy.Publisher('/ego_vehicle/purepursuit_control/ang_goal', PoseStamped, queue_size = 1)
vel_goal_pub = rospy.Publisher('/ego_vehicle/purepursuit_control/vel_goal', PoseStamped, queue_size = 1)
last_point_pub = rospy.Publisher('/ego_vehicle/purepursuit_control/last_point', PoseStamped, queue_size = 1)
plan_size    = 0

def construct_path():
    global plan
    global plan_size
    file_path = os.path.expanduser('../path/odom_final.csv')

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    plan_size = len(plan)


global ang_lookahead_dist
global vel_lookahead_dist

ang_lookahead_dist = 1
vel_lookahead_dist = 2

def pose_callback(data):
    global seq
    global plan
    global plan_size
    global ang_lookahead_dist
    global vel_lookahead_dist
    global last_point_index

    if (last_point_index-ang_lookahead_dist) >= data.data:
    	pose_index = (data.data + ang_lookahead_dist) % plan_size
    else:
	pose_index = last_point_index
    goal                    = PoseStamped()
    goal.header.seq         = seq
    goal.header.stamp       = rospy.Time.now()
    goal.header.frame_id    = frame_id
    goal.pose.position.x    = plan[pose_index][0]
    goal.pose.position.y    = plan[pose_index][1]

    ang_goal_pub.publish(goal)
    print('next goal:',goal.pose.position.x,goal.pose.position.y)

    if (last_point_index-vel_lookahead_dist) >= data.data:
    	pose_index = (data.data + vel_lookahead_dist) % plan_size
    else:
	pose_index = last_point_index

    goal                    = PoseStamped()
    goal.header.seq         = seq
    goal.header.stamp       = rospy.Time.now()
    goal.header.frame_id    = frame_id
    goal.pose.position.x    = plan[pose_index][0]
    goal.pose.position.y    = plan[pose_index][1]

    seq = seq + 1

    vel_goal_pub.publish(goal)

def last_point_callback(data):
    global last_point_index
    last_point_index = data.data
    global plan
    goal                    = PoseStamped()
    goal.pose.position.x    = plan[last_point_index][0]
    goal.pose.position.y    = plan[last_point_index][1]
    last_point_pub.publish(goal)
    print('last point',goal.pose.position)
    print('index',last_point_index)

if __name__ == '__main__':
    try:
        rospy.init_node('nearest_goal_isolator', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()
        rospy.Subscriber('/ego_vehicle/PP_control/index_last_point', Int64, last_point_callback)
        rospy.Subscriber('/ego_vehicle/PP_control/index_nearest_point', Int64, pose_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
