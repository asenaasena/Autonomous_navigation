#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import subprocess

# Define the launch files and their coordinate ranges
launch_files = {
    'mueavi_os_1': [(662549.479083,5771392.96869), (662570.423967,5771248.65301)],
    'mueavi_os_2': [(10,0), (20,10)],
    'mueavi_os_3': [(20,0), (30,10)],
    'mueavi_os_4': [(30,0), (40,10)],
    'mueavi_os_5': [(40,0), (50,10)],
    'mueavi_os_6': [(50,0), (60,10)],
    'mueavi_os_7': [(60,0), (70,10)],
    'mueavi_os_8': [(70,0), (80,10)]
}


# Define a dictionary to hold the running processes of the launch files
running_processes = {}

# Define a callback function to handle the odometry messages
def odom_callback(msg):
    global running_processes
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Check which launch files should be running
    for launch_file, coord_range in launch_files.items():
        if coord_range[0][0] <= x <= coord_range[1][0] and coord_range[0][1] <= y <= coord_range[1][1]:
            # Launch the launch file if it's not already running
            if launch_file not in running_processes:
                running_processes[launch_file] = subprocess.Popen(['roslaunch mueavi_tf ', launch_file + '.launch'])
                print("run success")
        else:
            # Stop the launch file if it's running
            if launch_file in running_processes:
                running_processes[launch_file].terminate()
                del running_processes[launch_file]
                print("not success")

if __name__ == '__main__':
    rospy.init_node('launch_files_based_on_odometry')

    # Subscribe to the odometry topic
    rospy.Subscriber('/gps/odom', Odometry, odom_callback)

    # Spin the node
    rospy.spin()
