#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def odom_callback(odom):
    x = odom.pose.pose.orientation.x
    y = odom.pose.pose.orientation.y
    z = odom.pose.pose.orientation.z
    w = odom.pose.pose.orientation.w
    roll, pitch, yaw = euler_from_quaternion([x, y, z, w])
    print("Current heading angle:", yaw)
    #rospy.loginfo('x = {}, y = {}, z = {}, w = {}'.format(x,y,z, w))
    
def veh_pose():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('veh_loc', anonymous=True)
    dt = 0.1
    rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        veh_pose()
    except rospy.ROSInterruptException:
        pass


