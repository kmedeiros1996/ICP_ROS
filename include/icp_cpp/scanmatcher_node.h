/*Scanmatcher node header file
* Author - Kyle M. Medeiros
*/


#ifndef SCANNODE_H
#define SCANNODE_H
//std
#include <string.h>
//ROS
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

// Third Party
#include <eigen3/Eigen/Dense>



#include "icp_cpp/icp.h"

class ScanMatcherNode {
public:
  ScanMatcherNode(ros::NodeHandle &n);
  Eigen::MatrixXd ConvertInputScan(const sensor_msgs::LaserScan::ConstPtr& input_scan);
  sensor_msgs::PointCloud2 ConvertOutputCloud(const Eigen::MatrixXd pc_matrix);
  void PublishTransform(const sensor_msgs::PointCloud2 &prev_scan, const sensor_msgs::PointCloud2 &curr_scan);
  void ProcessScanMatch(const sensor_msgs::LaserScan::ConstPtr& input_scan);

private:
  bool has_previous_scan_{false};
  Eigen::MatrixXd previous_scan_;
  sensor_msgs::PointCloud2 prev_scan_pc_;
  ICP icp_;
  ros::Publisher prev_scan_publisher_;
  ros::Publisher curr_scan_publisher_;
  int ts_sec_{0};
  int ts_nsec_{0};
  std::string frame_id_{"null"};
};

#endif //SCANNODE_H
