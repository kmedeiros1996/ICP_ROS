/*



*/


#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <iostream>
#include "icp_cpp/icp.h"
#include "icp_cpp/scanmatcher_node.h"
#include <string.h>
#include <eigen3/Eigen/Dense>
#include <laser_geometry/laser_geometry.h>
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include <cmath>


constexpr int kNumIterations = 100;

ScanMatcherNode::ScanMatcherNode(ros::NodeHandle &n)
:icp_(ICP(kNumIterations))
{
  has_previous_scan_ = false;
  prev_scan_publisher_ = n.advertise<sensor_msgs::PointCloud2>("prev_scan", 1000);
  curr_scan_publisher_ = n.advertise<sensor_msgs::PointCloud2>("curr_scan", 1000);
}

Eigen::MatrixXd ScanMatcherNode::ConvertInputScan(const sensor_msgs::LaserScan::ConstPtr& input_scan) {
  ts_sec_ = input_scan->header.stamp.sec;
  ts_nsec_ = input_scan->header.stamp.nsec;
  frame_id_ = input_scan->header.frame_id;
  sensor_msgs::PointCloud cloud;
  laser_geometry::LaserProjection laser_projector;
  laser_projector.projectLaser(*input_scan, cloud);
  Eigen::MatrixXd out_pointcloud_matrix = Eigen::MatrixXd::Zero(cloud.points.size(), 3);

  for (int i = 0; i < cloud.points.size(); i++) {
    auto pt = cloud.points[i];
    Eigen::Vector3d new_pt;
    new_pt<<pt.x,pt.y,pt.z;
    out_pointcloud_matrix.block<1,3>(i,0) = new_pt;
  }
  return out_pointcloud_matrix;
}

sensor_msgs::PointCloud2 ScanMatcherNode::ConvertOutputCloud(const Eigen::MatrixXd pc_matrix) {
  int num_rows = pc_matrix.rows();
  sensor_msgs::PointCloud2 cloud_msg;

  cloud_msg.header.stamp.sec = ts_sec_;
  cloud_msg.header.stamp.nsec = ts_nsec_;
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.height = 1;
  cloud_msg.width = 3;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);

  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(360);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  for (int i = 0; i < num_rows; ++i, ++iter_x, ++iter_y, ++iter_z) {
    Eigen::Vector3d point = pc_matrix.block<1,3>(i,0);
    *iter_x = point(0);
    *iter_y = point(1);
    *iter_z = point(2);
  }

  return cloud_msg;
}

void ScanMatcherNode::PublishTransform(const sensor_msgs::PointCloud2 &prev_scan, const sensor_msgs::PointCloud2 &curr_scan) {
  prev_scan_publisher_.publish(prev_scan);
  curr_scan_publisher_.publish(curr_scan);
}

void ScanMatcherNode::ProcessScanMatch(const sensor_msgs::LaserScan::ConstPtr& input_scan) {
  Eigen::MatrixXd current_scan = ConvertInputScan(input_scan);
  if (has_previous_scan_) {
    icp_.Run(current_scan, previous_scan_);
    Eigen::Matrix4d transform = icp_.GetTransform();
    int last_iters = icp_.GetPrevIterations();

    std::cout<<"Transform found after "<<last_iters<< " iterations: \n \n "<< transform<<std::endl;
    sensor_msgs::PointCloud2 curr_scan_pc = ConvertOutputCloud(icp_.GetTransformedPointcloud().transpose());
    PublishTransform(prev_scan_pc_, curr_scan_pc);
    previous_scan_ = icp_.GetTransformedPointcloud().transpose();
    prev_scan_pc_ = curr_scan_pc;
  } else {
    std::cout<<"First scan, initializing..."<<std::endl;
    has_previous_scan_ = true;
    previous_scan_ = current_scan;
  }
}
