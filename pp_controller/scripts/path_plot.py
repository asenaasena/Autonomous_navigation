#!/usr/bin/env python

import csv
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import os

rospy.init_node('path_plan')
path_pub = rospy.Publisher('/trajectory', Path, queue_size=1)

# Read path from CSV file
path = []
file_path = os.path.expanduser('/home/swapy/catkin_ws/src/pp_controller/path/odom_final.csv')
with open(file_path, 'r') as csv_file:
    reader = csv.reader(csv_file)
    for row in reader:
        x, y= map(float, row)
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        path.append(pose)
        print(path)

# Publish path
rate = rospy.Rate(10) # 10 Hz
while not rospy.is_shutdown():
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = 'trajectory' # Replace with your frame ID
    path_msg.poses = path
    path_pub.publish(path_msg)
    rate.sleep()


rospy.spin()

# rosrun tf static_transform_publisher 0 0 0 0 0 0 map trajectory 10













# import csv
# import rospy
# import os
# from std_msgs.msg import String
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Path

# global plan
# global seq
# plan = []
# seq = 0

# path = Path()
# def publish_waypoints():
#     global plan
#     global seq
#     file_path = os.path.expanduser('/home/carla/Desktop/catkin_ws/src/PP_controller/path/trajectory_name.csv')
    
#     # path = Path()
#     s = rospy.Rate(2)

#     path = Path()
#     with open(file_path, 'r') as f:
#         reader = csv.reader(f)
#         for row in reader:
#             x, y = map(float, row)
#             pose = PoseStamped()
#             pose.header.stamp = rospy.Time.now()
#             pose.header.frame_id = 'map'
#             pose.header.seq = seq
#             pose.pose.position.x = x
#             pose.pose.position.y = y
#             path.poses.append(pose)
            

#             seq = seq + 1
#             print('path',path.poses)
#             pub.publish(path)

# # rospy.init_node('waypoint_publisher', anonymous=True)
# # pub = rospy.Publisher('/trajectory', Path, queue_size=10)
# # publish_waypoints()

# # if __name__ == '__main__':
# #     rospy.spin()

# if __name__ == '__main__':
#     try:
#         rospy.init_node('waypoint_publisher', anonymous=True)
#         pub = rospy.Publisher('/trajectory', Path, queue_size=10)
        
#         publish_waypoints()
#         rospy.spin()
#         # rospy.sleep(s)
#     except rospy.ROSInterruptException:
#         pass
