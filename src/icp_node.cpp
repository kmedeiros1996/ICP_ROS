#include "icp_cpp/icp.h"
#include "icp_cpp/scanmatcher_node.h"
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"



int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_matcher");
  ros::NodeHandle n;
  ScanMatcherNode node(n);

  ros::Subscriber sub = n.subscribe("/scan", 1000, &ScanMatcherNode::ProcessScanMatch, &node);
  ros::spin();

  return 0;
}
