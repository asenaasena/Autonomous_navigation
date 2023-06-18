#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry

def odom_callback(odom):
    #get_caller_id(): Get fully resolved name of local node
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    z = odom.pose.pose.position.z
    angular_velocity = odom.twist.twist.angular.z
    heading_angle = angular_velocity * 0.1
    rospy.loginfo('x = {}, y = {}, z = {}, w = {}'.format(x,y,z, heading_angle))
    
def veh_pose():
    rospy.init_node('veh_loc', anonymous=True)
    dt = 0.1
    rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, odom_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        veh_pose()
    except rospy.ROSInterruptException:
        pass


