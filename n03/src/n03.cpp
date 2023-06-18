#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/filters/radius_outlier_removal.h>

#define FILTER_SIZE 30// 10
#define MINUS_FILTER_SIZE -30 //10
#define SEGMENTATION_THRESHOLD 0.05
#define RADIUSSEARCH 1.0
#define SETMINNEIGHBORSINRADIUS 15// 10 // 2 // 10



ros::Publisher pub;

sensor_msgs::PointCloud2 cloud_filtered;
pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCLPointCloud2:: Ptr cloud_bgd(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr cloud_fgd(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr cloud_bgd_passthrough_x(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr cloud_bgd_passthrough_y(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr cloud_bgd_passthrough_z(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr cloud_fgd_passthrough_x(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr cloud_fgd_passthrough_y(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr cloud_fgd_passthrough_z(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr cloud_bgd_downsampled(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr cloud_fgd_downsampled(new pcl::PCLPointCloud2);
pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_bgd_downsampled_xyz(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_fgd_downsampled_xyz(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>:: Ptr moving_objects_xyz(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCLPointCloud2:: Ptr cloud_background(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr cloud_foreground(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr moving_objects(new pcl::PCLPointCloud2);
pcl::PCLPointCloud2:: Ptr moving_objects_filtered(new pcl::PCLPointCloud2);


void cloud_cb (const pcl::PCLPointCloud2ConstPtr& input)
{
    ros::Time last_time = ros::Time::now();

//fill in the background cloud data
    pcl::PCDReader reader_bgd;
    //reader_bgd.read("/home/carla/Desktop/catkin_ws/src/2021_yongmin/n06/data/bgd_06.pcd", *cloud_bgd);
    reader_bgd.read("/home/carla/Desktop/catkin_ws/src/2021_yongmin/n03/data/new_bcg_3.pcd", *cloud_bgd);
    //reader_bgd.read("/home/carla/Desktop/catkin_ws/src/2021_yongmin/ICP/data/aligned_data/n_aligned_5.pcd", *cloud_bgd);
    

//background passthrough filter 
    pcl::PassThrough<pcl::PCLPointCloud2> pass_bgd_x;
    pass_bgd_x.setInputCloud (cloud_bgd);
    pass_bgd_x.setFilterFieldName ("x");
    pass_bgd_x.setFilterLimits(MINUS_FILTER_SIZE , FILTER_SIZE);
    pass_bgd_x.filter (*cloud_bgd_passthrough_x);

    pcl::PassThrough<pcl::PCLPointCloud2> pass_bgd_y;
    pass_bgd_y.setInputCloud (cloud_bgd_passthrough_x);
    pass_bgd_y.setFilterFieldName ("y");
    pass_bgd_y.setFilterLimits(MINUS_FILTER_SIZE , FILTER_SIZE);
    pass_bgd_y.filter (*cloud_bgd_passthrough_y);

    pcl::PassThrough<pcl::PCLPointCloud2> pass_bgd_z;
    pass_bgd_z.setInputCloud (cloud_bgd_passthrough_y);
    pass_bgd_z.setFilterFieldName ("z");
    pass_bgd_z.setFilterLimits(-10.0 , -2.0);
    pass_bgd_z.filter (*cloud_bgd_passthrough_z);


//foreground passthrough filter 
    pcl::PassThrough<pcl::PCLPointCloud2> pass_fgd_x;
    pass_fgd_x.setInputCloud (input);
    pass_fgd_x.setFilterFieldName ("x");
    pass_fgd_x.setFilterLimits(MINUS_FILTER_SIZE , FILTER_SIZE);
    pass_fgd_x.filter (*cloud_fgd_passthrough_x);

    pcl::PassThrough<pcl::PCLPointCloud2> pass_fgd_y;
    pass_fgd_y.setInputCloud (cloud_fgd_passthrough_x);
    pass_fgd_y.setFilterFieldName ("y");
    pass_fgd_y.setFilterLimits(MINUS_FILTER_SIZE , FILTER_SIZE);
    pass_fgd_y.filter (*cloud_fgd_passthrough_y);

    pcl::PassThrough<pcl::PCLPointCloud2> pass_fgd_z;
    pass_fgd_z.setInputCloud (cloud_fgd_passthrough_y);
    pass_fgd_z.setFilterFieldName ("z");
    pass_fgd_z.setFilterLimits(-10.0 , -2.0);
    pass_fgd_z.filter (*cloud_fgd_passthrough_z);


//segment the difference between the downsampled background and foreground
    pcl::fromPCLPointCloud2(*cloud_fgd_passthrough_z,*cloud_fgd_downsampled_xyz);
    pcl::fromPCLPointCloud2(*cloud_bgd_passthrough_z,*cloud_bgd_downsampled_xyz);


    pcl::PCDWriter writer_bgd;
    writer_bgd.write<pcl::PointXYZ> ("background_interest.pcd", *cloud_bgd_downsampled_xyz,false);
	
    pcl::search::KdTree<pcl::PointXYZ>:: Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::SegmentDifferences<pcl::PointXYZ> sdiff;
    sdiff.setInputCloud(cloud_fgd_downsampled_xyz);
    sdiff.setTargetCloud(cloud_bgd_downsampled_xyz);
    sdiff.setSearchMethod(tree);
    sdiff.setDistanceThreshold(SEGMENTATION_THRESHOLD);
    sdiff.segment(*moving_objects_xyz);

//use radius_outlier_removal to remove outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(moving_objects_xyz);
    outrem.setRadiusSearch(RADIUSSEARCH);
    outrem.setMinNeighborsInRadius(SETMINNEIGHBORSINRADIUS);
    outrem.filter(*cloud_filtered_xyz);

    pcl::toROSMsg(*cloud_filtered_xyz,cloud_filtered);
    pub.publish (cloud_filtered);
    ros::Time current_time = ros::Time::now();
    ros::Duration diff = current_time - last_time;
    //std::cerr<<"foreground extraction time = "<<diff<<std::endl;

}


int
main (int argc, char** argv)
{
  // Initialize ROS
  std::cout <<"where are the messages";
  ros::init (argc, argv, "n03_detection_moving");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud

  // I am running a rosbag for test = compressed bag


  // Checkout the name of the topic that you are publishing

  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  //ros::Subscriber sub = nh.subscribe ("/node06/os1_cloud_node/points", 1, cloud_cb); // human drive data
  //ros::Subscriber sub = nh.subscribe ("/base06/ouster/points", 1, cloud_cb); // our recorded data
  //ros::Subscriber sub = nh.subscribe ("/base03/ouster/points", 1, cloud_cb); // our recorded data
  //input

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("n03_moving", 1);

  // Spin
  ros::spin ();
}






