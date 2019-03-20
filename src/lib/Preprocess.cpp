#include "ztx/Preprocess.h"

#include <tf/transform_datatypes.h>


namespace ztx {

void Preprocess::initialize_params(ros::NodeHandle& privateNode) {
    std::string downsample_method = privateNode.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = privateNode.param<double>("downsample_resolution", 0.1);

    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = voxelgrid;
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" <<std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
    }

    std::string outlier_removal_method = privateNode.param<std::string>("outlier_removal_method", "STATISTICAL");
    if(outlier_removal_method == "STATISTICAL") {
      int mean_k = privateNode.param<int>("statistical_mean_k", 20);
      double stddev_mul_thresh = privateNode.param<double>("statistical_stddev", 1.0);
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;
    } else if(outlier_removal_method == "RADIUS") {
      double radius = privateNode.param<double>("radius_radius", 0.8);
      int min_neighbors = privateNode.param<int>("radus_min_neighbors", 2);
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
    } else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }

    use_distance_filter = privateNode.param<bool>("use_distance_filter", true);
    distance_near_thresh = privateNode.param<double>("distance_near_thresh", 1.0);
    distance_far_thresh = privateNode.param<double>("distance_far_thresh", 100.0);

    base_link_frame = privateNode.param<std::string>("base_link_frame", "");
  }

bool Preprocess::setupROS(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
  initialize_params(privateNode);
  imu_sub = node.subscribe<sensor_msgs::Imu>("/gpsimu_driver/imu_data", 50, &Preprocess::handleIMUMessage, this);
  imu_pub = node.advertise<sensor_msgs::Imu>("/filtered_imu", 32);

  points_sub = node.subscribe("/velodyne_points", 64, &Preprocess::handleCloudMessage, this);
  points_pub = node.advertise<sensor_msgs::PointCloud2>("/filtered_points", 32);
}


//没有使用IMU提供下一时刻位姿的先验信息，而是利用IMU去除激光传感器在运动过程中非匀速（加减速）部分造成的误差（运动畸变）,以满足匀速运动的假设。
//接收imu消息，imu坐标系为x轴向前，y轴向右，z轴向上的左手坐标系
    void Preprocess::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn)
{
 // tf::Quaternion orientation;
  //将IMU数据中的四元数转换到定义的四元数中
 // tf::quaternionMsgToTF(imuIn->orientation, orientation);//将geometry_msgs::Quaternion转化为tf::Quaternion类型
  double roll, pitch, yaw;
 // tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);//通过tf::Quaternion,将四元数转化为欧拉角
  imu_pub.publish(imuIn);
}


void Preprocess::handleCloudMessage(pcl::PointCloud<PointT>::ConstPtr src_cloud) {
    if(src_cloud->empty()) {
      return;
    }

    // if base_link_frame is defined, transform the input cloud to the frame
    if(!base_link_frame.empty()) {
      if(!tf_listener.canTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0))) {
        std::cerr << "failed to find transform between " << base_link_frame << " and " << src_cloud->header.frame_id << std::endl;
      }

      tf::StampedTransform transform;
      tf_listener.waitForTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
      tf_listener.lookupTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), transform);

      pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
      pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
      transformed->header.frame_id = base_link_frame;
      transformed->header.stamp = src_cloud->header.stamp;
      src_cloud = transformed;
    }

    pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(src_cloud);
    filtered = downsample(filtered);
    filtered = outlier_removal(filtered);
    points_pub.publish(filtered);
  }


} 

