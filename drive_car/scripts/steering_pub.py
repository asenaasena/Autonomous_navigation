#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive

def steer_cmd():
	
    pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size =10)
    rospy.init_node('steering_pub', anonymous = True)
    rate= rospy.Rate(1)
    rospy.loginfo("Publisher node is started, now publishing msgs")
        
    while not rospy.is_shutdown():
         msg = AckermannDrive()
         msg.speed = 1.0
         msg.steering_angle = 0.5
         pub.publish(msg)
         rospy.loginfo(msg.speed)
         rospy.loginfo(msg.steering_angle)
         rate.sleep()


if __name__ == '__main__':
    try:
        steer_cmd()
    except rospy.ROSInterruptException:
        pass
