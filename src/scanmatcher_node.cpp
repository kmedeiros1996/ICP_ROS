#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
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

ScanMatcherNode::ScanMatcherNode(ros::NodeHandle &n)
:icp(ICP(100))
{
  has_previous_scan = false;
  prev_scan_publisher = n.advertise<sensor_msgs::PointCloud2>("prev_scan", 1000);
  curr_scan_publisher = n.advertise<sensor_msgs::PointCloud2>("curr_scan", 1000);
}

Eigen::MatrixXd ScanMatcherNode::convert_input_scan(const sensor_msgs::LaserScan::ConstPtr& input_scan)
{


  ts_sec = input_scan->header.stamp.sec;
  ts_nsec = input_scan->header.stamp.nsec;
  frame_id = input_scan->header.frame_id;


  sensor_msgs::PointCloud cloud;
  laser_geometry::LaserProjection laser_projector;
  laser_projector.projectLaser(*input_scan, cloud);
  Eigen::MatrixXd out_pointcloud_matrix = Eigen::MatrixXd::Zero(360, 3);

  for (int i = 0; i < cloud.points.size(); i++)
  {

    auto pt = cloud.points[i];
    if (std::isnan(float(pt.x)) || std::isnan(float(pt.y)) || std::isnan(float(pt.z)))
    {
      std::cout<<"NAN encountered! "<<pt.x<<" "<<pt.y<<" "<<pt.z<<std::endl;
    }
    if (std::isinf(float(pt.x)) || std::isinf(float(pt.y)) || std::isinf(float(pt.z)))
    {
      std::cout<<"INF encountered! "<<pt.x<<" "<<pt.y<<" "<<pt.z<<std::endl;
    }

    Eigen::Vector3d new_pt;

    new_pt<<pt.x,pt.y,pt.z;

    out_pointcloud_matrix.block<1,3>(i,0) = new_pt;
  }

  return out_pointcloud_matrix;
}



sensor_msgs::PointCloud2 ScanMatcherNode::convert_output_cloud(const Eigen::MatrixXd pc_matrix)
{

  int num_rows = pc_matrix.rows();

  sensor_msgs::PointCloud2 cloud_msg;

  cloud_msg.header.stamp.sec = ts_sec;
  cloud_msg.header.stamp.nsec = ts_nsec;
  cloud_msg.header.frame_id = frame_id;

  cloud_msg.height = 1;
  cloud_msg.width = 3;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);

  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(360);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  for (int i = 0; i < num_rows; ++i, ++iter_x, ++iter_y, ++iter_z)
  {
    Eigen::Vector3d point = pc_matrix.block<1,3>(i,0);
    *iter_x = point(0);
    *iter_y = point(1);
    *iter_z = point(2);
  }

  return cloud_msg;
}



void ScanMatcherNode::publish_transform(const sensor_msgs::PointCloud2 &prev_scan, const sensor_msgs::PointCloud2 &curr_scan)
{
  prev_scan_publisher.publish(prev_scan);
  curr_scan_publisher.publish(curr_scan);
}

void ScanMatcherNode::process_scan_match(const sensor_msgs::LaserScan::ConstPtr& input_scan)
{


  Eigen::MatrixXd current_scan = convert_input_scan(input_scan);

  if (has_previous_scan)
  {

    icp.run_scan_matcher(current_scan, previous_scan);
    Eigen::Matrix4d transform = icp.get_transformation();
    int last_iters = icp.get_last_iterations();

    std::cout<<"Transform found after "<<last_iters<< " iterations: \n \n "<< transform<<std::endl;


  }
  else
  {
    std::cout<<"First scan, initializing..."<<std::endl;
    has_previous_scan = true;
  }

  sensor_msgs::PointCloud2 curr_scan_pc = convert_output_cloud(icp.get_transformed_pointcloud().transpose());

  publish_transform(prev_scan_pc, curr_scan_pc);
  previous_scan = current_scan;
  prev_scan_pc = curr_scan_pc;


}
