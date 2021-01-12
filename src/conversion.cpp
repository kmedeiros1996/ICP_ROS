//ROS
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>

// Third Party
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ICP
#include "icp_cpp/conversion.h"

Eigen::MatrixXd PointCloudToMatrix(const sensor_msgs::PointCloud& input_cloud) {
  const int num_pts = input_cloud.points.size();
  Eigen::MatrixXd out_pointcloud_matrix{Eigen::MatrixXd::Zero(num_pts, 3)};

  for (int i = 0; i < num_pts; i++) {
    auto pt = input_cloud.points[i];
    Eigen::VectorXd new_pt(3);
    new_pt<<pt.x,pt.y,pt.z;
    out_pointcloud_matrix.row(i) = new_pt;
  }

  return out_pointcloud_matrix;
}

Eigen::MatrixXd PointCloudToMatrix(const sensor_msgs::PointCloudConstPtr& input_cloud) {
  return PointCloudToMatrix(*input_cloud);
}

Eigen::MatrixXd LaserScanToMatrix(const sensor_msgs::LaserScanConstPtr& input_scan) {
  sensor_msgs::PointCloud cloud;
  laser_geometry::LaserProjection laser_projector;
  laser_projector.projectLaser(*input_scan, cloud);
  return PointCloudToMatrix(cloud);
}

Eigen::MatrixXd PointCloud2ToMatrix(const sensor_msgs::PointCloud2ConstPtr& input_cloud) {
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*input_cloud, cloud);
  return PointCloudToMatrix(cloud);
}

sensor_msgs::PointCloud2 MatrixToPointCloud2(const Eigen::MatrixXd& pc_matrix, std::string frame_id) {
  const int num_rows = pc_matrix.rows();
  sensor_msgs::PointCloud2 cloud_msg;

  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = frame_id;
  cloud_msg.height = 1;
  cloud_msg.width = 3;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);

  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(num_rows);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");


  for (int i = 0; i < num_rows; ++i, ++iter_x, ++iter_y, ++iter_z) {
    Eigen::Vector3d point = pc_matrix.row(i);
    *iter_x = point(0);
    *iter_y = point(1);
    *iter_z = point(2);
  }
  return cloud_msg;
}

std_msgs::Float64MultiArray MatrixToMultiArray(const Eigen::Matrix4d& matrix) {
  std_msgs::Float64MultiArray out;

  int rows = matrix.rows();
  int cols = matrix.cols();

  for (int i = 0; i < cols; i++) {
    out.layout.dim.push_back(std_msgs::MultiArrayDimension());
  }
  out.layout.dim[0].label = "x";
  out.layout.dim[1].label = "y";

  out.layout.dim[0].size = rows;
  out.layout.dim[1].size = cols;

  out.layout.dim[0].stride = rows * cols;
  out.layout.dim[1].stride = cols;
  std::vector<double> data (rows*cols, 0);

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      data[i*cols + j] = matrix(i, j);
    }
  }

  out.data = data;
  return out;
}

Eigen::MatrixXd MultiArrayToMatrix(const std_msgs::Float64MultiArray& matrix) {
  int rows = matrix.layout.dim[0].size;
  int cols = matrix.layout.dim[1].size;
  Eigen::MatrixXd out(rows, cols);

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      out(i, j) = matrix.data[i*cols + j];
    }
  }

  return out;
}

Eigen::MatrixXd OdometryToMatrix(const nav_msgs::Odometry& odom) {
  Eigen::MatrixXd out = Eigen::MatrixXd::Zero(4, 4);
  double x = odom.pose.pose.position.x;
  double y = odom.pose.pose.position.y;
  double z = odom.pose.pose.position.z;

  double qx = odom.pose.pose.orientation.x;
  double qy = odom.pose.pose.orientation.y;
  double qz = odom.pose.pose.orientation.z;
  double qw = odom.pose.pose.orientation.w;
  Eigen::Quaterniond q (qw, qx, qy, qz);

  out.block<3,1>(0,3) << x, y, z;
  out.block<3,3>(0,0) = q.toRotationMatrix();
  return out;
}
