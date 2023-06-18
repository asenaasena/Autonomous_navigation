from std_msgs.msg import String, Float32, Bool
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

import numpy as np
import math
import rospy

obstacle_threshold_distance = 20
obstacle_detection_lateral = 5

blind_spot_distance = 30
dt = 0.1

class TopicData:
    def __init__(self):
        self.odo_pos = None
        self.bbox_pos = None
        self.bbox_pos_arr = []
        self.bbox_arr = []
        self.vehicle_position_lidar = None
        self.publisher_vehicle =  rospy.Publisher("/ego_vehicle_bb",BoundingBox, queue_size = 10  ) 
        self.publisher_vehicle_distance =  rospy.Publisher("/ego_vehicle_distance_odom",Float32,queue_size = 10  ) 
        self.obstacle_detection_warning =  rospy.Publisher("/obstacle_detection",Bool,queue_size = 1  ) 
        self.blind_spot_warning =  rospy.Publisher("/blind_spot",Bool,queue_size = 1  ) 
        self.publisher_kalman_filter = rospy.Publisher("/ego_vehicle_position_kalman_filtered",PointStamped, queue_size = 10  ) 
        #self.publisher_gnss = rospy.Publisher("/gnss_publisher",Odometry, queue_size = 10  ) 
        self.is_blind_spot = False

 # Kalman Filter Parameters
        self.x = np.array([0, 0, 0, 0])  # initial state estimate
        self.P = np.diag([1, 1, 1, 1])  # initial covariance matrix
        self.Q = np.diag([0.1, 0.1, 0.1, 0.1])  # process noise covariance matrix
        self.R_odom = np.diag([0.5, 0.5])  # measurement noise covariance matrix for odometer data
        self.R_lidar = np.diag([0.1, 0.1])  # measurement noise covariance matrix for lidar data
        self.frame_id = None

    # state transition matrix
        self.F = np.array([[1, 0, 1, 0],
                           [0, 1, 0, 1],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])  


    # observation matrix for odometer data
        self.H_odom = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 0]])  


        self.H_lidar = np.array([[1, 0, 0, 0],
                                 [0, 1, 0, 0]])  # observation matrix for lidar data


        self.I = np.eye(4)  # 4x4 identity matrix
        
    def predict(self, dt):
        # state prediction step
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.x = np.matmul(self.F, self.x)
        self.P = np.matmul(np.matmul(self.F, self.P), self.F.T) + self.Q
        
    def update_odom(self, z):
        # odometer data update step
        S = np.matmul(np.matmul(self.H_odom, self.P), self.H_odom.T) + self.R_odom
        K = np.matmul(np.matmul(self.P, self.H_odom.T), np.linalg.inv(S))
        self.x = self.x + np.matmul(K, (z - np.matmul(self.H_odom, self.x)))
        self.P = np.matmul((self.I - np.matmul(K, self.H_odom)), self.P)
        
    def update_lidar(self, z):
        # lidar data update step
        S = np.matmul(np.matmul(self.H_lidar, self.P), self.H_lidar.T) + self.R_lidar
        K = np.matmul(np.matmul(self.P, self.H_lidar.T), np.linalg.inv(S))
        self.x = self.x + np.matmul(K, (z - np.matmul(self.H_lidar, self.x)))
        self.P = np.matmul((self.I - np.matmul(K, self.H_lidar)), self.P)

    def callback_3(self, data):
        # This function will be called whenever a new message is received on topic1
        # Store the latest data in latest_data_1    
        for box in data.boxes:
            self.transform_3(box)
            self.bbox_pos_arr.append(self.bbox_pos)
            self.bbox_arr.append(box)

    def transform_3(self, box):

        a = [box.pose.position.x,
                        box.pose.position.y, box.pose.position.z]

        # # From pole 3 to mueavi
        transform_3 = np.array([[-0.98825, -0.14978, 0.03056, 216.11301],
        [0.14964, -0.98872, -0.00684, 378.99963] ,
        [0.03124, -0.00218, 0.99951, 1.91038],
        [0.00000, 0.00000, 0.00000, 1.00000]])

        position_ = np.array([a[0] , a[1] , a[2]  , 1])
        res = np.matmul(transform_3, position_)
        self.bbox_pos = [res[0], res[1], res[2]]  # this is transformed version

    def determine_ego_vehicle(self):

        min_so_far = float("Inf")
        min_index = -5
        index = 0

        for box in self.bbox_pos_arr:
            # assume z coordinates is 0
            # find l2 distance between the odomoetry data and the center of the boxes
            total_diff = (box[0] -self.odo_pos[0] ) *(box[0] -self.odo_pos[0] ) + (box[1] -self.odo_pos[1] ) *(box[1] -self.odo_pos[1] )
            if total_diff <min_so_far :
                min_so_far = total_diff
                min_index = index                 
            index += 1
        
        return min_so_far, min_index


    def determine_ego_vehicle_publish(self):

        if len(self.bbox_pos_arr)!=0 and self.bbox_pos is not None and self.odo_pos is not None:

            # first determine the ego vehicle
            min_dist_so_far, min_index = self.determine_ego_vehicle()

            if math.sqrt(min_dist_so_far) < blind_spot_distance: #vehicle is not in the blind spot
                self.publisher_vehicle_distance.publish(math.sqrt(min_dist_so_far)) 
                self.publisher_vehicle.publish(self.bbox_arr[min_index]) 
                self.vehicle_position_lidar = self.bbox_arr[min_index].pose.position
                self.frame_id =  self.bbox_arr[min_index].header.frame_id
                self.blind_spot_warning.publish(False)
                self.is_blind_spot = False
                self.detect_obstacle()

            else: #vehicle is in the blind spot, and could not be seen by the lidar
                self.blind_spot_warning.publish(True)
                self.vehicle_position_lidar = None
                self.is_blind_spot = True

                # If it is not detected by the lidar, assign the location of gnss
                ego_vehicle_bb_message = BoundingBox()
                ego_vehicle_bb_message.pose.position.x = self.odo_pos[0]
                ego_vehicle_bb_message.pose.position.y = self.odo_pos[1]
                ego_vehicle_bb_message.pose.position.z = self.odo_pos[2]

                self.publisher_vehicle.publish(ego_vehicle_bb_message) 

            self.bbox_pos_arr = []
        

    def callback_2(self, data): # odometry
        # This function will be called whenever a new message is received on topic2
        # Store the latest data in latest_data_2
        self.determine_ego_vehicle_publish()
        
        position = data.pose.pose.position

        # This part is from global to the mueavi coordinates
        pos_x = position.x - 662338.291085
        pos_y = position.y - 5771078.29175
        self.odo_pos = [pos_x, pos_y, position.z]

        print("/boxs3self.odo_pos/",self.odo_pos)
        self.Kalman_filter()
    
    def detect_obstacle(self ):

        for box in self.bbox_pos_arr:                        
            # if the obstacle is in front and in the range of obstacle detection range, both laterally and in terms of l2 range
            if ( ((box[1] -self.vehicle_position_lidar.y ) > 0) and (((box[1] -self.vehicle_position_lidar.y ) *(box[1] -self.vehicle_position_lidar.y )) <obstacle_threshold_distance)
            and (np.absolute(box[0] -self.vehicle_position_lidar.x ) < obstacle_detection_lateral)) :
                self.obstacle_detection_warning.publish(True)


    # assuming constant time steps and the constant velocity, 2-D Kalman Filter Implementation
    def Kalman_filter(self):
        
        # read odometry data from sensors
        if self.odo_pos is not None:
            odom_data = np.array([self.odo_pos[0],self.odo_pos[1]]) # 2 dimensional
            self.predict(dt)
            
            # update Kalman filter
            if not(self.is_blind_spot) and self.vehicle_position_lidar is not None:
                lidar_data = np.array([self.vehicle_position_lidar.x,self.vehicle_position_lidar.y])
                self.update_lidar(lidar_data)
            self.update_odom(odom_data)

            # get estimated position
            point = PointStamped()
            point.point.x= self.x[0]
            point.point.y = self.x[1]
            point.point.z = 0
            point.header.frame_id = self.frame_id
            self.publisher_kalman_filter.publish(point)

        
def main():
    rospy.init_node('listener', anonymous=True)
    topic_data = TopicData()
    # Subscribe to both topics and connect them to the respective callbacks
    rospy.Subscriber("/boxs3", BoundingBoxArray, topic_data.callback_3) # bounding boxes coming from eu_cluster3 node 
    rospy.Subscriber('/gps/odom', Odometry, topic_data.callback_2) # odom data will be used to determine the ego vehicle and also used in Kalman filter
    rospy.spin()


if __name__ == '__main__':
    main()
