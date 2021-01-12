/*
* Scanmatcher driver code header file
* Author - Kyle M. Medeiros
*/

#ifndef ICP_DRIVER_H
#define ICP_DRIVER_H

//std
#include <string>
#include <memory>

//ROS
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// Third Party
#include <Eigen/Dense>

// ICP
#include "icp_cpp/icp_utils.h"
#include "icp_cpp/program_options.h"
#include "icp_cpp/icp.h"

enum EScanMatchMode { MODE_SEQUENTIAL, MODE_A_TO_B };

/*
* Scan match driver which initializes ROS publishers/subscribers and runs in sequential mode or A To B Mode.
*/
class ScanMatchDriver {
public:

  /*
  * @brief constructor for the scan match driver.
  * initializes publishers / subscribers according to program options and calls ros::spin().
  */
  ScanMatchDriver(const ProgramOptions& options);

private:

  /*
  * @brief initialize ROS publishers on the heap according to program options.
  * By default, will always initialize a publisher for the transformation matrix given by ICP
  * @param options ProgramOptions struct used to determine which publishers to initialize
  */
  void InitializePublishers(const ProgramOptions& options);

  /*
  * @brief initialize ROS subscribers on the heap according to program options
  * Depending on mode, initializes one or two pointcloud subscribers with configurable message type (LaserScan, PointCloud, PointCloud2)
  * @param options ProgramOptions struct used to determine which publishers to initialize
  */
  void InitializeSubscribers(const ProgramOptions& options);

  /*
  * @brief Callback for sensor_msgs::LaserScan.
  * Converts message to Eigen::MatrixXd and passes along to ProcessScanA or ProcessScanB depending on topic name.
  * @param input_scan const pointer to input sensor_msgs::LaserScan message
  * @param topic used to determine which ProcessScan method to send output matrix to. Bound to a topic during initialization via a lambda function.
  */
  void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& input_scan, const std::string &topic);

  /*
  * @brief Callback for sensor_msgs::PointCloud.
  * Converts message to Eigen::MatrixXd and passes along to ProcessScanA or ProcessScanB depending on topic name.
  * @param input_cloud const pointer to input sensor_msgs::PointCloud message
  * @param topic used to determine which ProcessScan method to send output matrix to. Bound to a topic during initialization via a lambda function.
  */
  void PointCloudCallback(const sensor_msgs::PointCloudConstPtr& input_cloud, const std::string &topic);

  /*
  * @brief Callback for sensor_msgs::PointCloud2.
  * Converts message to Eigen::MatrixXd and passes along to ProcessScanA or ProcessScanB depending on topic name.
  * @param input_cloud const pointer to input sensor_msgs::PointCloud2 message
  * @param topic used to determine which ProcessScan method to send output matrix to. Bound to a topic during initialization via a lambda function.
  */
  void PC2Callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud, const std::string &topic);

  /*
  * @brief Initial Guess Float64MultiArray callback.
  * Sets ICP initial guess to the matrix received on the initial_guess_topic.
  * @param initial_guess 4D transformation matrix encoding initial transform for Scan A
  */
  void InitialGuessMatrixCallback(const std_msgs::Float64MultiArray& initial_guess);

  /*
  * @brief Initial Guess nav_msgs::Odometry callback.
  * If this is the first odom message, sets a reference inverse pose matrix
  * which all subsequent odom matrices are transformed with respect to
  * in order to obtain an initial guess transform.
  * Sets ICP initial guess to the odometry message received on the initial_guess_topic.
  * @param initial_guess odometry message with initial guess transform info for scan A
  */
  void InitialGuessOdometryCallback(const nav_msgs::Odometry& initial_guess);

  /*
  * @brief Process an input scan received on the scan_a input topic.
  * Used only in sequential mode.
  * - If first scan, sets Scan B to pc_matrix
  * - Otherwise, Scan A is set to pc_matrixScan A and matched against Scan B
  * - Post-alignment Scan A becomes Scan B
  *
  * @param pc_matrix (n_points * dim) matrix of pointcloud data
  */
  void ProcessScanSequential(const Eigen::MatrixXd& pc_matrix);

  /*
  * @brief Process an input scan received on the scan_a input topic.
  * Used only in A to B mode.
  * - Sets Scan A to pc_matrix
  * - if Scan B is set, match Scan A against Scan B
  *
  * @param pc_matrix (n_points * dim) matrix of pointcloud data
  */
  void ProcessScanA(const Eigen::MatrixXd& pc_matrix);

  /*
  * @brief Process an input scan received on the scan_b input topic.
  * Used only in A to B mode.
  * Sets Scan B to pc_matrix and builds a KD Tree from the points.
  *
  * @param pc_matrix (n_points * dim) matrix of pointcloud data
  */
  void ProcessScanB(const Eigen::MatrixXd& pc_matrix);

  /*
  * @brief Runs ICP in regular mode. After ICP converges, will publish the transform / transformed Scan A
  */
  void RunICPRegularMode();

  /*
  * @brief Runs ICP in stepwise mode.
  * Publishes the current transformed scan on every iteration of ICP and waits for stepwise_time_interval_ seconds.
  * After ICP converges, will publish the transform.
  */
  void RunICPStepwiseMode();

  /*
  * @brief method to create a pointer to a subscriber on the heap
  * @param topic topic for subsciber to listen to. Bound to a lambda callback which is passed to the output subscriber.
  * @param msg_type input ROS message format  (LaserScan, PointCloud, PointCloud2)
  * @param rate subscriber rate
  */
  std::unique_ptr<ros::Subscriber> MakeSubscriber(const std::string& topic, const std::string& msg_type, int rate);

  EScanMatchMode mode_{MODE_SEQUENTIAL};                            // Driver mode enum, either Sequential or A-To-B
  bool has_both_scans_{false};                                      // Flag for if both scans are available to run ICP on
  Eigen::MatrixXd scan_a_;                                          // Scan A
  Eigen::MatrixXd transformed_scan_a_;                              // Scan A, post-transform
  Eigen::MatrixXd scan_b_;                                          // Scan B

  ICP icp_;                                                         // Scan Matcher class
  std::string frame_id_{"null"};                                    // TF frame id
  ros::NodeHandle node_handle_;                                     // ROS node handle
  std::string input_a_topic_;                                       // Topic to receive Scan A on (used in sequential and A-to-B mode)
  std::string input_b_topic_;                                       // Topic to receive Scan B on (A-to-B mode only)
  bool show_each_step_;                                             // Flag to run ICP in show_each_step mode
  Eigen::Matrix4d init_pose_{Eigen::Matrix4d::Zero()};              // 4D transformation matrix representing initial odom pose
  bool has_init_pose_{false};
  std::unique_ptr<ros::Duration> stepwise_time_interval_;           // Pointer to ros::Duration time interval to wait in between iterations (in stepwise scan mode)
  std::unique_ptr<ros::Publisher> transform_publisher_{nullptr};    // Pointer to transformation matrix publisher
  std::unique_ptr<ros::Publisher> scan_a_publisher_{nullptr};       // Pointer to scan a publisher
  std::unique_ptr<ros::Publisher> step_scan_a_publisher_{nullptr};  // Pointer to stepwise scan a publisher
  std::unique_ptr<ros::Publisher> trans_scan_a_publisher_{nullptr}; // Pointer to post-transform scan a publisher
  std::unique_ptr<ros::Publisher> scan_b_publisher_{nullptr};       // Pointer to scan b publisher
  std::unique_ptr<ros::Subscriber> scan_a_subscriber_{nullptr};     // Pointer to scan a subscriber
  std::unique_ptr<ros::Subscriber> scan_b_subscriber_{nullptr};     // Pointer to scan b subscriber
  std::unique_ptr<ros::Subscriber> guess_subscriber_{nullptr};      // Pointer to initial guess subscriber
};

#endif //ICP_DRIVER_H
