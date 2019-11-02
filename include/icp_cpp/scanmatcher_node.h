#ifndef SCANNODE_H
#define SCANNODE_H

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "icp_cpp/icp.h"
#include <string.h>
#include <eigen3/Eigen/Dense>
#include <laser_geometry/laser_geometry.h>
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"

class ScanMatcherNode
{
private:
  bool has_previous_scan;
  Eigen::MatrixXd previous_scan;
  sensor_msgs::PointCloud2 prev_scan_pc;
  ICP icp;
  ros::Publisher prev_scan_publisher;
  ros::Publisher curr_scan_publisher;
  int ts_sec;
  int ts_nsec;
  std::string frame_id;



public:
  ScanMatcherNode(ros::NodeHandle &n);
  Eigen::MatrixXd convert_input_scan(const sensor_msgs::LaserScan::ConstPtr& input_scan);
  sensor_msgs::PointCloud2 convert_output_cloud(const Eigen::MatrixXd pc_matrix);
  void publish_transform(const sensor_msgs::PointCloud2 &prev_scan, const sensor_msgs::PointCloud2 &curr_scan);
  void process_scan_match(const sensor_msgs::LaserScan::ConstPtr& input_scan);

};




#endif //SCANNODE_H
