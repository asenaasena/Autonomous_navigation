#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix


def steering_cmd(message):
    #get_caller_id(): Get fully resolved name of local node
    print('x = {}'.format(message.latitude))
    print('y = {}'.format(message.longitude))
    
    
def steering_sub():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('steering_sub', anonymous=True)

    rospy.Subscriber("/gps/fix", NavSatFix, steering_cmd)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        steering_sub()
    except rospy.ROSInterruptException:
        pass


