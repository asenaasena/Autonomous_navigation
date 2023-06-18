#include <ros/ros.h>

#include <iostream>
#include <thread>
// PCL specific includes
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std::literals::chrono_literals;
using namespace sensor_msgs;
using namespace message_filters;
using namespace rosgraph_msgs;

ros::Publisher pub;
std_msgs::Header point_cloud_header_;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input,
              const geometry_msgs::PoseStampedConstPtr& pose_input) {
  ros::Time last_time = ros::Time::now();
  int itr = 0;

  // Loading first scan.
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader_input;
  reader_input.read("/path/src/matching_test/data/nissan_4.pcd", *input_cloud);

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr target_2(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*input, *target_2);
  pcl::fromPCLPointCloud2(*target_2, *target_cloud);

  if (target_cloud->size() > 100) {
    // Filtering input scan to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.3, 0.3, 0.3);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_chassis;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_front;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_rear;

    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> ndt_chassis;
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> ndt_front;
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> ndt_rear;
    // Set initial alignment estimate found using robot odometry.

    Eigen::MatrixXd transform_1(4, 4);
    Eigen::MatrixXd osgb36_transform(4, 4);

    // node.3
    // transform_1 << -0.98825, -0.14978, 0.03056, 216.11301,
    //                 0.14964, -0.98872, -0.00684, 378.99963,
    //                 0.03124, -0.00218, 0.99951, 1.91038,
    //                 0.00000, 0.00000, 0.00000, 1.00000; // node3
    // node.5
    // transform_1 <<0.99991,	0.00743	,-0.01143	,217.89305,
    //               -0.00749,	0.99996,	-0.00519,	318.53171,
    //               0.01139	,0.00527,	0.99992	,2.52351,
    //               0.00000	,0.00000	,0.00000	,1.00000;//
    //               node05
    // //node.6
    // transform_1 << 0.99530, -0.09255, -0.02863, 223.394104,
    //                0.09275, 0.99567, 0.00550, 264.11304,
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
    transform_1 << -0.34297, -0.93930, 0.00978, 138.69794, 0.93912, -0.34309,
        -0.01836, 22.06364, 0.02060, 0.00289, 0.99978, 2.96138, 0, 0, 0,
        1;  // node15
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

    osgb36_transform << 1, 0, 0, 493898.42, 0, 1, 0, 241772.0313, 0, 0, 1,
        107.43235, 0, 0, 0, 1;
    Eigen::Matrix3f R(3, 3);

    // R(0,0)=-0.990;
    // R(0,1)=0.024;
    // R(0,2)=-0.215;
    // R(1,0)=-0.023;
    // R(1,1)=-1.000;
    // R(1,2)=-0.009;
    // R(2,0)=-0.215;
    // R(2,1)=-0.004;
    // R(2,2)=0.997;
    double pi = 3;

    R(0, 0) = cos(pi);
    R(0, 1) = -sin(pi);
    R(0, 2) = 0;
    R(1, 0) = sin(pi);
    R(1, 1) = cos(pi);
    R(1, 2) = 0;
    R(2, 0) = 0;
    R(2, 1) = 0;
    R(2, 2) = 1;

    // R(0,0)= 0.878055930138;
    // R(0,1)= -0.478412479162;
    // R(0,2)= 0.011800684966;
    // R(1,0)= 0.477535635233;
    // R(1,1)= 0.877524673939;
    // R(1,2)= 0.043704822659;
    // R(2,0)= -0.031264323741;
    // R(2,1)= -0.032740030438;
    // R(2,2)= 0.998974800110;

    Eigen::Vector3f T_chassis(3);
    Eigen::Vector3f T_front(3);
    Eigen::Vector3f T_rear(3);

    // chassis center
    T_chassis(0) = pose_input->pose.position.x;
    T_chassis(1) = pose_input->pose.position.y - 0.3;
    T_chassis(2) = -3.51;  // pose_input->pose.position.z-0.5; //+ 0.624502;

    // rear axle center
    T_rear(0) = pose_input->pose.position.x;
    T_rear(1) = pose_input->pose.position.y - 2;  // + 1.4;
    T_rear(2) = -3.51;  // pose_input->pose.position.z-0.5; //+ 0.51;

    // front axle center
    T_front(0) = pose_input->pose.position.x;
    T_front(1) = pose_input->pose.position.y + 2;  // -2;
    T_front(2) = -3.51;                            //+ 0.51;

    Eigen::Matrix4f init_guess_chassis;
    init_guess_chassis.setIdentity();
    init_guess_chassis.block<3, 3>(0, 0) = R;
    init_guess_chassis.block<3, 1>(0, 3) = T_chassis;

    Eigen::Matrix4f init_guess_front;
    init_guess_front.setIdentity();
    init_guess_front.block<3, 3>(0, 0) = R;
    init_guess_front.block<3, 1>(0, 3) = T_front;

    Eigen::Matrix4f init_guess_rear;
    init_guess_rear.setIdentity();
    init_guess_rear.block<3, 3>(0, 0) = R;
    init_guess_rear.block<3, 1>(0, 3) = T_rear;

    int transform_eps = 0.84;
    int stepsize = 0.13;
    int resolution = 0.64;

    // Setting scale dependent NDT parameters
    // reference point: centre of the chassis bottom
    // Setting minimum transformation difference for termination condition.
    ndt_chassis.setTransformationEpsilon(transform_eps);
    // Setting maximum step size fr More-Thuente line search.
    ndt_chassis.setStepSize(stepsize);
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt_chassis.setResolution(resolution);
    // Setting max number of registration iterations.
    ndt_chassis.setMaximumIterations(300);

    // Setting point cloud to be aligned.
    ndt_chassis.setInputSource(filtered_cloud);
    // Setting point cloud to be aligned to.
    ndt_chassis.setInputTarget(target_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_chassis(
        new pcl::PointCloud<pcl::PointXYZ>);
    ndt_chassis.align(*output_cloud_chassis, init_guess_chassis);
    Eigen::Matrix4f ndt_chassis_transformation =
        ndt_chassis.getFinalTransformation();

    // reference point: centre of the front axle
    // Setting minimum transformation difference for termination condition.
    ndt_front.setTransformationEpsilon(transform_eps);
    // Setting maximum step size fr More-Thuente line search.
    ndt_front.setStepSize(stepsize);
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt_front.setResolution(resolution);
    // Setting max number of registration iterations.
    ndt_front.setMaximumIterations(300);

    // Setting point cloud to be aligned.
    ndt_front.setInputSource(filtered_cloud);
    // Setting point cloud to be aligned to.
    ndt_front.setInputTarget(target_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_front(
        new pcl::PointCloud<pcl::PointXYZ>);
    ndt_front.align(*output_cloud_front, init_guess_front);
    Eigen::Matrix4f ndt_front_transformation =
        ndt_front.getFinalTransformation();

    // reference point: centre of the rear axle
    // Setting minimum transformation difference for termination condition.
    ndt_rear.setTransformationEpsilon(transform_eps);
    // Setting maximum step size fr More-Thuente line search.
    ndt_rear.setStepSize(stepsize);
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt_rear.setResolution(resolution);
    // Setting max number of registration iterations.
    ndt_rear.setMaximumIterations(300);

    // Setting point cloud to be aligned.
    ndt_rear.setInputSource(filtered_cloud);
    // Setting point cloud to be aligned to.
    ndt_rear.setInputTarget(target_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_rear(
        new pcl::PointCloud<pcl::PointXYZ>);
    ndt_rear.align(*output_cloud_rear, init_guess_rear);
    Eigen::Matrix4f ndt_rear_transformation = ndt_rear.getFinalTransformation();

    point_cloud_header_ = input->header;
    sensor_msgs::PointCloud2 cloud_mesh;

    double score[3] = {ndt_front.getFitnessScore(),
                       ndt_chassis.getFitnessScore(),
                       ndt_rear.getFitnessScore()};

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> matching_pc(3);
    matching_pc[0] = output_cloud_front;
    matching_pc[1] = output_cloud_chassis;
    matching_pc[2] = output_cloud_rear;

    std::vector<Eigen::Matrix4f> finaltransform(3);
    finaltransform[0] = ndt_chassis_transformation;
    finaltransform[1] = ndt_front_transformation;
    finaltransform[2] = ndt_rear_transformation;

    // std::cerr << matching_pc[0] << endl;
    // std::cerr << ndt_chassis_transformation(0,3) <<" "
    // <<ndt_chassis_transformation(1,3)<<" "\
        <<ndt_chassis_transformation(2,3)<<" "<<ndt_chassis_transformation(3,3)<<
    endl;
    // std::cerr << ndt_chassis_transformation << endl;

    double temp = 5000;
    int k = 0;

    for (int i = 0; i < 3; i++) {
      if (score[i] <= temp) {
        temp = score[i];
        k = i;
      }
    }

    pcl::toROSMsg(*matching_pc[k], cloud_mesh);

    cloud_mesh.header = point_cloud_header_;

    pub.publish(cloud_mesh);

    // std::cerr<<finaltransform[k](0,3) << endl;

    pcl::PointXYZ center;
    pcl::computeCentroid(*matching_pc[k], center);

    Eigen::MatrixXd center_matrix(4, 1);
    center_matrix << center.x, center.y, center.z, 1;

    Eigen::MatrixXd final_center_tf(4, 1);

    final_center_tf = transform_1 * center_matrix;

    Eigen::MatrixXd final_transform(4, 1);
    final_transform << finaltransform[k](0, 3), finaltransform[k](1, 3),
        finaltransform[k](2, 3), finaltransform[k](3, 3);

    Eigen::MatrixXd final_transform_tf(4, 1);
    Eigen::MatrixXd osgb_transform_tf(4, 1);

    final_transform_tf = transform_1 * final_transform;
    osgb_transform_tf = osgb36_transform * final_transform_tf;
    std::cerr << ros::Time::now() << " " << final_transform_tf(0, 0) << " "
              << final_transform_tf(1, 0) << " " << final_transform_tf(2, 0)
              << std::endl;
    // std::cerr <<osgb_transform_tf(0,0)<<" "<< osgb_transform_tf(1,0) << "
    // "<<osgb_transform_tf(2,0) <<std::endl;
  };

  ros::Time current_time = ros::Time::now();
  ros::Duration diff = current_time - last_time;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "matching_test");
  ros::NodeHandle nh;

  message_filters::Subscriber<rosgraph_msgs::Clock> clock(nh, "/clock", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(
      nh, "/cluster_pc", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(
      nh, "/box_pose", 1);
  TimeSynchronizer<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> sync(
      pc_sub, pose_sub, 1);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));

  pub = nh.advertise<sensor_msgs::PointCloud2>("mesh", 1);

  ros::spin();
}
