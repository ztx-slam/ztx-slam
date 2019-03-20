#include "Twist.h"
#include "nanoflann_pcl.h"

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
"

namespace ztx
{

  /** \brief Implementation of the LOAM laser odometry component.
   *
   */
  class lidarodom 
  {
  public:
    
    /** \brief Setup component.
     *
     * @param node the ROS node handle
     * @param privateNode the private ROS node handle
     */
     bool setup(ros::NodeHandle& node,ros::NodeHandle& privateNode);

    /** \brief Handler method for a new IMU transformation information.
     *
     * @param laserCloudFullResMsg the new IMU transformation information message
     */
    void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg);


    /** \brief Process incoming messages in a loop until shutdown (used in active mode). */
    void spin();

    /** \brief Try to process buffered data. */
    void process();

  protected:
    /** \brief Reset flags, etc. */
    void reset();

    /** \brief Check if all required information for a new processing step is available. */
    bool hasNewData();

    /** \brief Publish the current result via the respective topics. */
    void publishResult();

  private:
    pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }  

  private:
    uint16_t _ioRatio;       ///< ratio of input to output frames

    ros::Time _timeImuTrans;               ///< time of current IMU transformation information

    bool _newImuTrans;                ///< flag if a new IMU transformation information cloud has been received

    nav_msgs::Odometry _lidarodometryMsg;       ///< laser odometry message
    tf::StampedTransform _lidarodometryTrans;   ///< laser odometry transformation

    
    ros::Publisher _publidarodometry;         ///< laser odometry publisher
    tf::TransformBroadcaster _tfBroadcaster;  ///< laser odometry transform broadcaster

    ros::Subscriber points_sub;      ///< sharp corner cloud message subscriber
    ros::Subscriber _subImuTrans;               ///< IMU transformation information message subscriber
  };

} 
