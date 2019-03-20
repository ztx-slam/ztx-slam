#include <ros/ros.h>
#include "ztx/lidarodom.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidarodom_node");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  ztx::lidarodom laserOdom(0.1);

  if (laserOdom.setup(node, privateNode)) {
    // initialization successful
    laserOdom.spin();
  }

  return 0;
}

