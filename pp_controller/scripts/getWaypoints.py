#!/usr/bin/env python

import rospy
import csv
import os
from nav_msgs.msg import Path # Change the message type as per your topic
from std_msgs.msg import Bool # Import Bool message type from std_msgs

trajectory_closed = Bool() # Initialize the boolean variable
trajectory_closed.data = False # Set the initial value to False

def callback(data):
    # Callback function to receive the data
    with open("/home/carla/Desktop/catkin_ws/src/PP_controller/path/trajectory_name.csv", 'w') as file:
        for pose in data.poses:
            file.write(str(pose.pose.position.x) + ',' + str(pose.pose.position.y) +  '\n')
        file.close()
        trajectory_closed.data = True # Set the boolean variable to True when the file is closed
        pub.publish(trajectory_closed) # Publish the boolean variable to the new topic
        trajectory_closed.data = False # Set the boolean variable to True when the file is closed
        pub.publish(trajectory_closed) # Publish the boolean variable to the new topic
    print('Trajectory updated')

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, callback) # Change the topic name and message type as per your requirement
    pub = rospy.Publisher('trajectory_closed', Bool, queue_size=10) # Create the new topic for publishing the boolean variable
    rospy.spin()

if __name__ == '__main__':
    listener()
