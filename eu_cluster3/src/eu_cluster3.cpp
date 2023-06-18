//#pragma once

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <fstream>
#include <iostream>
#include <vector>

#define LEAF_SIZE 0.05
#define MIN_CLUSTER_SIZE 100  // 10

#define MAX_CLUSTER_SIZE 5000

using namespace std;

ros::Subscriber sub_point_cloud_;
ros::Publisher pub_bounding_boxs_;
ros::Publisher cluster_pose;
ros::Publisher cluster_cloud;
std_msgs::Header point_cloud_header_;
std::vector<double> seg_distance_, cluster_distance_;
using namespace std;
using namespace Eigen;

struct Detected_Obj {
  jsk_recognition_msgs::BoundingBox bounding_box_;

  pcl::PointXYZ min_point_;
  pcl::PointXYZ max_point_;
  pcl::PointXYZ centroid_;

  pcl::PointCloud<pcl::PointXYZ> pc_cluster_;
};

void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr out,
                      double leaf_size);

void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                         std::vector<Detected_Obj> &obj_list);

void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                     double in_max_cluster_distance,
                     std::vector<Detected_Obj> &obj_list);

void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

void publish_cloud(const ros::Publisher &in_publisher,
                   const pcl::PointCloud<pcl::PointXYZ> in_cloud_to_publish,
                   const std_msgs::Header &in_header);

void saveData(string fileName, MatrixXd matrix) {
  const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ",", "\n");

  ofstream file(fileName);
  if (file.is_open()) {
    file << matrix.format(CSVFormat);
    file.close();
  }
}

void publish_cloud(const ros::Publisher &in_publisher,
                   const pcl::PointCloud<pcl::PointXYZ> in_cloud_to_publish,
                   const std_msgs::Header &in_header) {
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(in_cloud_to_publish,
                cloud_msg);  // in_cloud_to_publish to cloud_msg
  cloud_msg.header = in_header;
  in_publisher.publish(cloud_msg);
}

void voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size) {
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(in);
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.filter(*out);
}

void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                     double in_max_cluster_distance,
                     std::vector<Detected_Obj> &obj_list) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);

  // create 2d pc
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*in_pc, *cloud_2d);
  // make it flat
  for (size_t i = 0; i < cloud_2d->points.size(); i++) {
    cloud_2d->points[i].z = 0;
  }

  if (cloud_2d->points.size() > 0) tree->setInputCloud(cloud_2d);

  std::vector<pcl::PointIndices> local_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
  euclid.setInputCloud(cloud_2d);
  euclid.setClusterTolerance(5);
  euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
  euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
  euclid.setSearchMethod(tree);
  euclid.extract(local_indices);

  for (size_t i = 0; i < local_indices.size(); i++) {
    // the structure to save one detected object
    Detected_Obj obj_info;

    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (auto pit = local_indices[i].indices.begin();
         pit != local_indices[i].indices.end(); ++pit) {
      // fill new colored cluster point by point
      pcl::PointXYZ p;
      p.x = in_pc->points[*pit].x;
      p.y = in_pc->points[*pit].y;
      p.z = in_pc->points[*pit].z;

      obj_info.pc_cluster_.points.push_back(in_pc->points[*pit]);

      obj_info.centroid_.x += p.x;
      obj_info.centroid_.y += p.y;
      obj_info.centroid_.z += p.z;

      if (p.x < min_x) min_x = p.x;
      if (p.y < min_y) min_y = p.y;
      if (p.z < min_z) min_z = p.z;
      if (p.x > max_x) max_x = p.x;
      if (p.y > max_y) max_y = p.y;
      if (p.z > max_z) max_z = p.z;
    }

    // min, max points
    obj_info.min_point_.x = min_x;
    obj_info.min_point_.y = min_y;
    obj_info.min_point_.z = min_z;

    obj_info.max_point_.x = max_x;
    obj_info.max_point_.y = max_y;
    obj_info.max_point_.z = max_z;

    // calculate centroid, average
    if (local_indices[i].indices.size() > 0) {
      obj_info.centroid_.x /= local_indices[i].indices.size();
      obj_info.centroid_.y /= local_indices[i].indices.size();
      obj_info.centroid_.z /= local_indices[i].indices.size();
    }

    // calculate bounding box
    double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
    double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
    double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

    obj_info.bounding_box_.header = point_cloud_header_;

    obj_info.bounding_box_.pose.position.x =
        obj_info.min_point_.x + length_ / 2;
    obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
    obj_info.bounding_box_.pose.position.z =
        obj_info.min_point_.z + height_ / 2;

    obj_info.bounding_box_.dimensions.x =
        ((length_ < 0) ? -1 * length_ : length_);
    obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
    obj_info.bounding_box_.dimensions.z =
        ((height_ < 0) ? -1 * height_ : height_);

    obj_list.push_back(obj_info);
    obj_info.pc_cluster_.points.clear();
  }
}

void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                         std::vector<Detected_Obj> &obj_list) {
  // cluster the pointcloud according to the distance of the points using
  // different thresholds (not only one for the entire pc) in this way, the
  // points farther in the pc will also be clustered

  // 0 => 0-15m d=0.5
  // 1 => 15-30 d=1
  // 2 => 30-45 d=1.6
  // 3 => 45-60 d=2.1
  // 4 => >60   d=2.6

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(5);

  for (size_t i = 0; i < segment_pc_array.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    segment_pc_array[i] = tmp;
  }

  for (size_t i = 0; i < in_pc->points.size(); i++) {
    pcl::PointXYZ current_point;
    current_point.x = in_pc->points[i].x;
    current_point.y = in_pc->points[i].y;
    current_point.z = in_pc->points[i].z;

    float origin_distance =
        sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

    if (origin_distance >= 40) {
      continue;
    }

    if (origin_distance < seg_distance_[0]) {
      segment_pc_array[0]->points.push_back(current_point);
    } else if (origin_distance < seg_distance_[1]) {
      segment_pc_array[1]->points.push_back(current_point);
    } else if (origin_distance < seg_distance_[2]) {
      segment_pc_array[2]->points.push_back(current_point);
    } else if (origin_distance < seg_distance_[3]) {
      segment_pc_array[3]->points.push_back(current_point);
    } else {
      segment_pc_array[4]->points.push_back(current_point);
    }
  }

  std::vector<pcl::PointIndices> final_indices;
  std::vector<pcl::PointIndices> tmp_indices;

  for (size_t i = 0; i < segment_pc_array.size(); i++) {
    cluster_segment(segment_pc_array[i], cluster_distance_[i], obj_list);
  }
}

void cluster_callback_03(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr) {
  std::cerr << "cluster 3 is running" << endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);

  point_cloud_header_ = in_cloud_ptr->header;

  pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

  // downsampling the point cloud before cluster
  voxel_grid(current_pc_ptr, filtered_pc_ptr, LEAF_SIZE);

  geometry_msgs::PoseStamped box_pose;
  box_pose.header = point_cloud_header_;

  std::vector<Detected_Obj> global_obj_list;

  cluster_by_distance(filtered_pc_ptr, global_obj_list);
  jsk_recognition_msgs::BoundingBoxArray bbox_array;

  for (size_t i = 0; i < global_obj_list.size(); i++) {
    bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
    box_pose.pose = global_obj_list[i].bounding_box_.pose;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(global_obj_list[i].pc_cluster_, cloud_msg);
    global_obj_list[i].pc_cluster_.points.clear();
    cloud_msg.header = point_cloud_header_;
    cluster_cloud.publish(cloud_msg);
    cluster_pose.publish(box_pose);
    // std::cerr<< global_obj_list[i].bounding_box_.pose.position.x<<endl;
    // std::cerr<<box_pose.pose.orientation<< endl;
  }

  bbox_array.header = point_cloud_header_;
  pub_bounding_boxs_.publish(bbox_array);

  Eigen::MatrixXd transform_1(4, 4);
  // Depending on the sensor block, one should uncomment specific values,
  // transform 1 corresponds to the node3
  // // node.3
  transform_1 << -0.98825, -0.14978, 0.03056, 216.11301, 0.14964, -0.98872,
      -0.00684, 378.99963, 0.03124, -0.00218, 0.99951, 1.91038, 0.00000,
      0.00000, 0.00000, 1.00000;  // node3
  // //node.5
  // transform_1 <<0.99991,	0.00743	,-0.01143	,217.89305,
  //                -0.00749,	0.99996,	-0.00519,	318.53171,
  //                0.01139	,0.00527,	0.99992	,2.52351,
  //                0.00000	,0.00000	,0.00000	,1.00000;//
  //                node05
  // // //node.6
  // transform_1 << 0.99530, -0.09255, -0.02863, 223.394104,
  //                 0.09275, 0.99567, 0.00550, 264.11304,
  //                0.02800, -0.00813, 0.99957, 2.85158,
  //                0.00000, 0.00000, 0.00000, 1.000000; //node06
  // node.11
  // transform_1 << 0.97018, 0.24206, -0.01273, 237.29823,
  //                -0.24203, 0.97026, 0.00375, 60.77664,
  //                0.01326, -0.00056, 0.99991, 3.63130,
  //                0.00000, 0.00000, 0.00000, 1.00000; //node11
  // node.13
  // transform_1 << 0.39885,	0.91700,	-0.00501,	206.98775,
  //                -0.91690	,0.39888	,0.01373,	31.14257,
  //                0.01459	,-0.00088,	0.99989,	3.61192,
  //                0,	0,	0,	1; //node13
  // node.15
  // transform_1 << -0.34297, -0.93930, 0.00978, 138.69794,
  //                0.93912, -0.34309, -0.01836, 22.06364,
  //                0.02060, 0.00289, 0.99978, 2.96138,
  //                0, 0, 0, 1; //node15
  // //node.18
  // transform_1 << 0.56748, 0.82337 ,-0.004397 ,29.10938,
  //                -0.82327 ,0.56749 ,0.01377, -16.75850,
  //                0.013830 ,-0.00420, 0.99990, 3.14369,
  //                0.00000, 0.00000, 0.00000, 1.00000; //node18
  // //node.19
  // transform_1 << -0.97483, -0.22263, 0.01173, 8.68959,
  //                 0.22257, -0.97490, -0.00650, 13.25823,
  //                 0.01288, -0.00373, 0.99991, 3.25088,
  //                 0, 0, 0, 1; //node19

  Eigen::MatrixXd transform_2(4, 1);
  // transform_2 <<  box_pose.pose.position.x + transform_1(0,3)
  // ,box_pose.pose.position.y + transform_1(1,3), box_pose.pose.position.z +
  // transform_1(2,3), 1;
  transform_2 << box_pose.pose.position.x, box_pose.pose.position.y,
      box_pose.pose.position.z, 1;

  Eigen::MatrixXd final_transform(4, 1);
  // Eigen::MatrixXd final_transform_transpose (3,1);

  // final_transform_transpose << final_transform (0,0), final_transform (1,0),
  // final_transform (2,0);

  final_transform = transform_1 * transform_2;

  saveData("matrix.csv", final_transform);

  // std::cerr <<final_transform(0,0) <<" "<< final_transform(1,0) <<"
  // "<<final_transform(2,0)<<std::endl; std::cerr << box_pose.pose.position <<
  // endl;

  // std::cerr << "box_pose = "<< box_pose.pose.position <<std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "euclidean_cluster");

  ros::NodeHandle nh;

  seg_distance_ = {10, 10, 10, 10};
  // cluster_distance_ = {1, 2.5, 2.5, 2.5, 2.5};
  cluster_distance_ = {1};

  sub_point_cloud_ = nh.subscribe("/n03_moving", 1, cluster_callback_03);

  pub_bounding_boxs_ =
      nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/boxs3", 1);
  cluster_pose = nh.advertise<geometry_msgs::PoseStamped>("box_pose3", 1);
  cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>("cluster_pc3", 1);

  ros::spin();
}
