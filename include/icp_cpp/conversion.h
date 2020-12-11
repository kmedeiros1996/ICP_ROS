#ifndef ICP_CONVERSION_H
#define ICP_CONVERSION_H

//ROS
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64MultiArray.h>

// Third Party
#include <Eigen/Dense>

/*----------------------Input Conversions------------------------*/
/*
* @brief turns a sensor_msgs::LaserScan into a sensor_msgs::PointCloud
* @param input_scan const pointer to an input laser scan
*/
sensor_msgs::PointCloud LaserScanToPointCloud(const sensor_msgs::LaserScanConstPtr& input_scan);

/*
* @brief turns a sensor_msgs::LaserScan into an Eigen::MatrixXd containing xyz point information
* @param input_scan input laser scan
*/
Eigen::MatrixXd LaserScanToMatrix(const sensor_msgs::LaserScanConstPtr& input_scan);

/*
* @brief turns a sensor_msgs::PointCloud into an Eigen::MatrixXd containing xyz point information
* @param input_cloud const pointer to an input PointCloud
*/
Eigen::MatrixXd PointCloudToMatrix(const sensor_msgs::PointCloudConstPtr& input_cloud);

/*
* @brief turns a sensor_msgs::PointCloud into an Eigen::MatrixXd containing xyz point information
* @param input_cloud an input PointCloud
*/
Eigen::MatrixXd PointCloudToMatrix(const sensor_msgs::PointCloud& input_cloud);

/*
* @brief turns a sensor_msgs::PointCloud2 into an Eigen::MatrixXd containing xyz point information
* @param input_cloud const pointer to an input PointCloud2
*/
Eigen::MatrixXd PointCloud2ToMatrix(const sensor_msgs::PointCloud2ConstPtr& input_cloud);

/*----------------------Output Conversions------------------------*/

/*
* @brief turns an Eigen::MatrixXd containing xyz point information into a PointCloud2.
* Timestamp information is filled in with ros::Time::now()
* @param pc_matrix matrix w/ pointcloud information
* @param frame_id tf frame id
*/
sensor_msgs::PointCloud2 MatrixToPointCloud2(const Eigen::MatrixXd& pc_matrix, std::string frame_id);

/*
* @brief turns an Eigen::MatrixXd into a Float64MultiArray of matching dimensions.
* @param matrix dynamic size matrix of doubles
*/
std_msgs::Float64MultiArray MatrixToMultiArray(const Eigen::MatrixXd& matrix);

#endif //ICP_CONVERSION_H
