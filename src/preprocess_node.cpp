#include <ros/ros.h>
#include "ztx/Preprocess.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "preprocess_node");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  ztx::Preprocess multiScan;

  if (multiScan.setupROS(node, privateNode)) {
    // initialization successful
    ros::spin();
  }

  return 0;
}

