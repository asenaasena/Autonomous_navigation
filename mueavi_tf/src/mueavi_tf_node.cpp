#include "mueavi_tf/mueavi_tf_node.hpp"

#include <tf2_ros/static_transform_broadcaster.h>

const std::vector<geometry_msgs::TransformStamped> csv_to_tf_msg(void) {
  // Read and parse transformations from csv
  std::string pkg_path;
  pkg_path = ros::package::getPath("mueavi_tf");
  io::CSVReader<14> in(pkg_path + "/data/mueavi_transformations.csv");
  std::string frame_id, child_frame_id;
  double O1, O2, O3;                          // Origin of the child frame
  double M1, M2, M3, M4, M5, M6, M7, M8, M9;  // Elements of rotation matrix

  std::vector<geometry_msgs::TransformStamped> tf_msgs{};

  while (in.read_row(frame_id, child_frame_id, O1, O2, O3, M1, M2, M3, M4, M5,
                     M6, M7, M8, M9)) {
    // Convert transformations to ROS tf2 messages
    tf2::Transform tf{};
    tf.setOrigin({O1, O2, O3});
    tf.setBasis({M1, M4, M7, M2, M5, M8, M3, M6,
                 M9});  // Matlab is column major c++ is row major!
    geometry_msgs::TransformStamped tf_msg{};
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = frame_id;       // parent frame id
    tf_msg.child_frame_id = child_frame_id;  // child frame id
    tf_msg.transform = tf2::toMsg(tf);
    tf_msgs.push_back(tf_msg);
  }
  return tf_msgs;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mueavi_tf");
  ros::NodeHandle nh("~");

  // Publish transformations
  static tf2_ros::StaticTransformBroadcaster tf_bcast{};

  tf_bcast.sendTransform(csv_to_tf_msg());
  ROS_INFO("Spinning...");

  ros::spin();

  return EXIT_SUCCESS;
}
