
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include "time_utils.h"

namespace ztx {

/** \brief Construct a new point cloud message from the specified information and publish it via the given publisher.
 *
 * @tparam PointT the point type
 * @param publisher the publisher instance
 * @param cloud the cloud to publish
 * @param stamp the time stamp of the cloud message
 * @param frameID the message frame ID
 */
template <typename PointT>
inline void publishCloudMsg(ros::Publisher& publisher,
                            const pcl::PointCloud<PointT>& cloud,
                            const ros::Time& stamp,
                            std::string frameID) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = frameID;
  publisher.publish(msg);
}


// ROS time adapters
inline Time fromROSTime(ros::Time const& rosTime)
{
  auto epoch = std::chrono::system_clock::time_point();
  auto since_epoch = std::chrono::seconds(rosTime.sec) + std::chrono::nanoseconds(rosTime.nsec);
  return epoch + since_epoch;
}

inline ros::Time toROSTime(Time const& time_point)
{
  return ros::Time().fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(time_point.time_since_epoch()).count());
}

} 
