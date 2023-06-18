print("  importing bar_fn, baz_fn from simon_pythonsubpackage1")
from simon_pythonsubpackage1 import bar_fn, baz_fn


import rospy
from std_msgs.msg import String
import rospy
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from sensor_msgs.msg import Imu
import math
from nav_msgs.msg import Odometry
import numpy as np


class TopicData:
    def __init__(self):
        self.odo_pos = None
        self.bbox_pos = None

    def callback_1(self, data):
        # This function will be called whenever a new message is received on topic1
        # Store the latest data in latest_data_1
        self.bbox_pos = [data.boxes[0].pose.position.x,
                         data.boxes[0].pose.position.y, data.boxes[0].pose.position.z]

        # # From pole 3 to mueavi

        transform_3 = np.array([[-0.98825, -0.14978, 0.03056, 216.11301],
         [0.14964, -0.98872, -0.00684, 378.99963] ,
         [0.03124, -0.00218, 0.99951, 1.91038],
         [0.00000, 0.00000, 0.00000, 1.00000]])

        # transform_3_partial = np.array([[-0.98825, -0.14978, 0.03056],
        # [0.14964, -0.98872, -0.0068] ,
        # [0.03124, -0.00218, 0.99951],
        # ])

		# uni teacher

        transform_3 = np.array([[0.98921401, -0.146337123, 0.006410111, 216.231931],[0.146333939, 0.989234778, 0.000965463, 378.97439] ,
         [-0.006482388, -1.70328E-05, 0.999978989, 1.5009],
         [0.00000, 0.00000, 0.00000, 1.00000]])


		# error =6-7-8
        transform_3 = np.array([[0.98921401, 0.146333939, -0.006482388, 216.231931],
         [-0.146337123, 0.989234778, -1.70328E-05, 378.97439] ,
         [0.006410111, 0.000965463, 0.999978989, 1.5009],
         [0.00000, 0.00000, 0.00000, 1.00000]])

        transform_3 = np.array([[0.999357, 0.00123154, 0.00123154, -0.257424],
         [-0.00133635, 0.999996, 0.00291036,  0.31199] ,
         [-0.035858, -0.0029564, 0.999353, 9.34567],
         [0.00000, 0.00000, 0.00000, 1.00000]])

        # transformation =transform_3_partial.T

        # transform_3 = np.array([[transformation[0][0], transformation[0][1], transformation[0][2], 216.231931],
        # [transformation[1][0], transformation[1][1], transformation[1][2], 378.97439] ,
        # [transformation[2][0], transformation[2][1], transformation[2][2], 1.5009],
        # [0.00000, 0.00000, 0.00000, 1.00000]])

        # transform_3 = np.array([[-0.98825, -0.14978, 0.03056, 216.11301],
        # [0.14964, -0.98872, -0.00684, 378.99963] ,
        # [0.03124, -0.00218, 0.99951, 1.91038],
        # [0.00000, 0.00000, 0.00000, 1.00000]])

        position_ = np.array([self.bbox_pos[0] + 216, self.bbox_pos[1] + 378, self.bbox_pos[2] +1.1 , 1])
        res = np.matmul(transform_3, position_)
        self.bbox_pos = [res[0], res[1], res[2]]

    def compute(self):

        total_diff = 0

        if self.bbox_pos is not None and self.odo_pos is not None:
            print("self.bbox_pos", self.bbox_pos)
            print("self.odo_pos", self.odo_pos)

            for first, second in zip(self.bbox_pos, self.odo_pos):
                diff = first - second
                total_diff += diff*diff
            print("Difference between topic1 and topic2:", math.sqrt(total_diff))
        else:
            print("Not enough data received on both topics")

    def callback_2(self, data):
        # This function will be called whenever a new message is received on topic2
        # Store the latest data in latest_data_2
        self.compute()

        position = data.pose.pose.position
        pos_x = position.x - 662338.291085
        pos_y = position.y - 5771078.29175

        self.odo_pos = [pos_x, pos_y, position.z]
        # position_ = np.array([pos_x, pos_y, position.z, 1])

        # transform_3 = np.array([[-0.98825, -0.14978, 0.03056, 216.11301],
        # [0.14964, -0.98872, -0.00684, 378.99963] ,
        # [0.03124, -0.00218, 0.99951, 1.91038],
        # [0.00000, 0.00000, 0.00000, 1.00000]])

        # transform_3 = np.array([[0.98921401, -0.14978, 0.006410111, 216.231931],
        # [0.146333939, 0.989234778, 0.000965463, 378.97439] ,
        # [-0.006482388, -1.70328E-05, 0.999978989, 1.5009],
        # [0.00000, 0.00000, 0.00000, 1.00000]])

        # #a =np.matmul(transform_3, transform_4)
        # res = np.matmul(transform_3, position_)
        # self.odo_pos = [res[0], res[1], res[2]]


def main():
    rospy.init_node('listener', anonymous=True)
    topic_data = TopicData()
# Subscribe to both topics and connect them to the respective callbacks
    rospy.Subscriber("/boxs", BoundingBoxArray, topic_data.callback_1)
    rospy.Subscriber('/gps/odom', Odometry, topic_data.callback_2)
    rospy.spin()


if __name__ == '__main__':
    main()
