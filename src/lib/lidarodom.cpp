#include "ztx/LaserOdometry.h"
#include "ztx/common.h"


namespace ztx
{



bool LaserOdometry::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
  {
  _publidarodometry = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);
  _subImuTrans = node.subscribe<sensor_msgs::PointCloud2>("/imu_trans", 5, &LaserOdometry::imuTransHandler, this);
  initialize_params(privateNode);

    points_sub = nh.subscribe("/filtered_points", 256, &ScanMatchingOdometryNodelet::cloud_callback, this);
    read_until_pub = nh.advertise<std_msgs::Header>("/scan_matching_odometry/read_until", 32);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 32);
  }

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if(!ros::ok()) {
      return;
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    Eigen::Matrix4f pose = matching(cloud_msg->header.stamp, cloud);
    publish_odometry(cloud_msg->header.stamp, cloud_msg->header.frame_id, pose);

    // In offline estimation, point clouds until the published time will be supplied 在离线估计中，提供点云直到发布时间。
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = points_topic;
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

  }

}

