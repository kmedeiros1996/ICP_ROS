//std
#include <iostream>
#include <string>
#include <cmath>
#include <memory>

// ROS
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// ICP
#include "icp_cpp/icp.h"
#include "icp_cpp/conversion.h"
#include "icp_cpp/icp_utils.h"
#include "icp_cpp/scanmatcher_driver.h"

ScanMatchDriver::ScanMatchDriver(const ProgramOptions& options)
:frame_id_(options.frame_id),
node_handle_(),
icp_(options.max_num_iterations),
has_both_scans_(false),
input_a_topic_(options.input_scan_a_topic),
input_b_topic_(options.input_scan_b_topic),
show_each_step_(options.show_each_step) {
  mode_ = (options.mode == "sequential") ? MODE_SEQUENTIAL : MODE_A_TO_B;
  std::cout<<"Starting ICP driver in "<<options.mode<<" mode"<<std::endl;

  if (show_each_step_) {
    std::cout<<"Running in debug SHOW_EACH_STEP mode. "<<std::endl;
    std::cout<<"Scans will be published after every iteration every "<<options.stepwise_time_interval<<"secs"<<std::endl;
    stepwise_time_interval_ = std::make_unique<ros::Duration>(options.stepwise_time_interval);
  }

  if (options.enable_publishing) {
    InitializePublishers(options);
  }

  if (options.enable_subscribing) {
    InitializeSubscribers(options);
    ros::spin();
  }
}

void ScanMatchDriver::InitializePublishers(const ProgramOptions& options) {

    transform_publisher_ = std::make_unique<ros::Publisher>(node_handle_.advertise<std_msgs::Float64MultiArray>(options.output_transform_topic, options.queue_size));
    std::cout<<"Initialized Transformation Matrix publisher on topic "<<options.output_transform_topic<< " with queue size "<<options.queue_size<<std::endl;

    if (options.pub_scan_a) {
      scan_a_publisher_ = std::make_unique<ros::Publisher>(node_handle_.advertise<sensor_msgs::PointCloud2>(options.output_scan_a_topic, options.queue_size));
      std::cout<<"Initialized Scan A publisher on topic "<<options.output_scan_a_topic<< " with queue size "<<options.queue_size<<std::endl;
    }

    if (options.pub_trans_scan_a) {
      trans_scan_a_publisher_ = std::make_unique<ros::Publisher>(node_handle_.advertise<sensor_msgs::PointCloud2>(options.output_trans_scan_a_topic, options.queue_size));
      std::cout<<"Initialized Transformed Scan A publisher on topic "<<options.output_trans_scan_a_topic<< " with queue size "<<options.queue_size<<std::endl;
    }

    if (options.pub_scan_b) {
      scan_b_publisher_ = std::make_unique<ros::Publisher>(node_handle_.advertise<sensor_msgs::PointCloud2>(options.output_scan_b_topic, options.queue_size));
      std::cout<<"Initialized Scan B publisher on topic "<<options.output_scan_b_topic<< " with queue size "<<options.queue_size<<std::endl;
    }

    if (options.show_each_step) {
      step_scan_a_publisher_ = std::make_unique<ros::Publisher>(node_handle_.advertise<sensor_msgs::PointCloud2>(options.output_stepwise_a_topic, options.queue_size));
      std::cout<<"Initialized Stepwise Scan A publisher on topic "<<options.output_stepwise_a_topic<< " with queue size "<<options.queue_size<<std::endl;
    }
}

std::unique_ptr<ros::Subscriber> ScanMatchDriver::MakeSubscriber(const std::string& topic, const std::string& msg_type, int rate) {
  if (msg_type == "laserscan"){
      boost::function<void(const sensor_msgs::LaserScanConstPtr&)> callback;
      callback = [this, topic](const sensor_msgs::LaserScanConstPtr& msg)-> void {this->LaserScanCallback(msg, topic);};
      return std::make_unique<ros::Subscriber>(node_handle_.subscribe(topic, rate, callback));
    } else if(msg_type == "pointcloud") {
      boost::function<void(const sensor_msgs::PointCloudConstPtr&)> callback;
      callback = [this, topic](const sensor_msgs::PointCloudConstPtr& msg)-> void {this->PointCloudCallback(msg, topic);};
      return std::make_unique<ros::Subscriber>(node_handle_.subscribe(topic, rate, callback));
    } else if(msg_type == "pointcloud2") {
      boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)> callback;
      callback = [this, topic](const sensor_msgs::PointCloud2ConstPtr& msg)-> void {this->PC2Callback(msg, topic);};
      return std::make_unique<ros::Subscriber>(node_handle_.subscribe(topic, rate, callback));
    } else if(msg_type == "multiarray"){
      return std::make_unique<ros::Subscriber>(node_handle_.subscribe(topic, rate, &ScanMatchDriver::InitialGuessMatrixCallback, this));
    } else if(msg_type == "odometry"){
      return std::make_unique<ros::Subscriber>(node_handle_.subscribe(topic, rate, &ScanMatchDriver::InitialGuessOdometryCallback, this));
    }else {
      std::string err_string = "Invalid input message type "+msg_type+"! ";
      throw std::runtime_error(err_string);
    }
}

void ScanMatchDriver::InitializeSubscribers(const ProgramOptions& options) {
    scan_a_subscriber_ = MakeSubscriber(input_a_topic_, options.input_scan_a_type, options.queue_size);
    std::cout<<"Receiving input scan A of type "<< options.input_scan_a_type<<" on topic "<<options.input_scan_a_topic<< " with queue size "<<options.queue_size<<std::endl;

    guess_subscriber_ = MakeSubscriber(options.initial_guess_topic, options.initial_guess_type, options.queue_size);
    std::cout<<"Receiving input initial guess of type "<< options.initial_guess_type<<" on topic "<<options.initial_guess_topic<< " with queue size "<<options.queue_size<<std::endl;

    if (mode_ == MODE_A_TO_B) {
    scan_b_subscriber_ = MakeSubscriber(input_b_topic_, options.input_scan_b_type, options.queue_size);
      std::cout<<"Receiving input scan B of type "<< options.input_scan_b_type<<" on topic "<<options.input_scan_b_topic<< " with queue size "<<options.queue_size<<std::endl;
    }
}

void ScanMatchDriver::SetInitialGuess(const Eigen::Matrix4d& guess) {
  icp_.SetInitialGuess(guess);
  has_initial_guess_ = true;
}

void ScanMatchDriver::LaserScanCallback(const sensor_msgs::LaserScanConstPtr& input_scan, const std::string &topic) {
  if (topic == input_a_topic_) {
    if (mode_ == MODE_SEQUENTIAL) {
      ProcessScanSequential(LaserScanToMatrix(input_scan));
    } else {
      ProcessScanA(LaserScanToMatrix(input_scan));
    }
  } else if (topic == input_b_topic_) {
    ProcessScanB(LaserScanToMatrix(input_scan));
  }
}

void ScanMatchDriver::PointCloudCallback(const sensor_msgs::PointCloudConstPtr& input_cloud, const std::string &topic) {
  if (topic == input_a_topic_) {
    if (mode_ == MODE_SEQUENTIAL) {
      ProcessScanSequential(PointCloudToMatrix(input_cloud));
    } else {
      ProcessScanA(PointCloudToMatrix(input_cloud));
    }
  } else if (topic == input_b_topic_) {
    ProcessScanB(PointCloudToMatrix(input_cloud));
  }
}

void ScanMatchDriver::PC2Callback(const sensor_msgs::PointCloud2ConstPtr& input_cloud, const std::string &topic) {
  if (topic == input_a_topic_) {
    if (mode_ == MODE_SEQUENTIAL) {
      ProcessScanSequential(PointCloud2ToMatrix(input_cloud));
    } else {
      ProcessScanA(PointCloud2ToMatrix(input_cloud));
    }
  } else if (topic == input_b_topic_) {
    ProcessScanB(PointCloud2ToMatrix(input_cloud));
  }
}

void ScanMatchDriver::InitialGuessMatrixCallback(const std_msgs::Float64MultiArray& initial_guess) {
  SetInitialGuess(MultiArrayToMatrix(initial_guess));
}

void ScanMatchDriver::InitialGuessOdometryCallback(const nav_msgs::Odometry& odom) {
  Eigen::Matrix4d new_odom = OdometryToMatrix(odom);
  Eigen::Matrix4d relative_odometry = prev_odom_pose_inv_* new_odom;

  SetInitialGuess(relative_odometry);
  prev_odom_pose_inv_ = new_odom.inverse();
}

void ScanMatchDriver::ProcessScanSequential(const Eigen::MatrixXd& pc_matrix) {
  icp_.SetScanA(pc_matrix);

  if (scan_a_publisher_ != nullptr) {
    scan_a_publisher_->publish(MatrixToPointCloud2(icp_.GetInputScanA(), frame_id_));
  }
  if (!has_initial_guess_) {
    std::cout<<"Waiting for initial guess..."<<std::endl;
    return;
  }
  if (has_both_scans_) {
    if (show_each_step_) {
      RunICPStepwiseMode();
    } else {
      RunICPRegularMode();
    }
    ProcessScanB(icp_.GetTransformedScanA());
  } else {
    std::cout<<"Sequential Scan Match: initializing first scan..."<<std::endl;
    ProcessScanB(pc_matrix);
  }
}

void ScanMatchDriver::ProcessScanA(const Eigen::MatrixXd& pc_matrix) {
  icp_.SetScanA(pc_matrix);
  if (scan_a_publisher_ != nullptr) {
    scan_a_publisher_->publish(MatrixToPointCloud2(icp_.GetInputScanA(), frame_id_));
  }
  if (step_scan_a_publisher_ != nullptr) {
    step_scan_a_publisher_->publish(MatrixToPointCloud2(icp_.GetTransformedScanA(), frame_id_));
  }

  if (!has_initial_guess_) {
    std::cout<<"Waiting for initial guess..."<<std::endl;
    return;
  }

  if (has_both_scans_) {
    if (show_each_step_) {
      RunICPStepwiseMode();
    } else {
      RunICPRegularMode();
    }
  } else {
    std::cout<<"A to B Scan Match: Scan A has been overridden. Awaiting scan B to match against..."<<std::endl;
  }
}

void ScanMatchDriver::ProcessScanB(const Eigen::MatrixXd& pc_matrix) {
  std::cout<<"Setting Scan B and building kd tree...";
  icp_.SetScanB(pc_matrix);
  std::cout<<"Done!"<<std::endl;
  has_both_scans_ = true;

  if (scan_b_publisher_!= nullptr) {
    scan_b_publisher_->publish(MatrixToPointCloud2(icp_.GetScanB(), frame_id_));
  }
}

void ScanMatchDriver::RunICPRegularMode() {
  std::cout<<"Waiting for ICP to converge...";
  icp_.MatchScanAToScanB();
  std::cout<<"Done!"<<std::endl;

  Eigen::Matrix4d transform = icp_.GetTransform();
  std::cout<<"Transform found after "<<icp_.GetPrevIterations()<< " iterations:"<<std::endl<<transform<<std::endl;
  util::PrintTransform(transform);

  // Reset the initial guess flag so that a new transform can be provided (or absolute odom pose to compute a transform from)
  has_initial_guess_ = false;

  if (transform_publisher_ != nullptr) {
    transform_publisher_->publish(MatrixToMultiArray(transform));
  }

  if (trans_scan_a_publisher_ != nullptr) {
    trans_scan_a_publisher_->publish(MatrixToPointCloud2(icp_.GetTransformedScanA(), frame_id_));
  }
}

void ScanMatchDriver::RunICPStepwiseMode() {
  std::cout<<"Running scan match in stepwise mode."<<std::endl;

  if (scan_a_publisher_ != nullptr) {
    scan_a_publisher_->publish(MatrixToPointCloud2(icp_.GetInputScanA(), frame_id_));
  }

  double prev_error = 0.0;
  double mean_error = 0.0;
  double diff;
  int iters;

  for (iters = 1; iters <= icp_.GetMaxIterations(); iters++) {
    mean_error = icp_.RunOneIteration();

    if (step_scan_a_publisher_ != nullptr) {
      step_scan_a_publisher_->publish(MatrixToPointCloud2(icp_.GetTransformedScanA(), frame_id_));
    }

    diff = fabs(mean_error - prev_error);
    if (diff < icp_.GetTolerance()) {
      break;
    }
    prev_error = mean_error;

    if (stepwise_time_interval_ != nullptr) {
      stepwise_time_interval_->sleep();
    }
  }
  Eigen::Matrix4d transform = icp_.ComputeFinalTransform();

  std::cout<<"Done!"<<std::endl;
  std::cout<<"Transform found after "<<iters<< " iterations:"<<std::endl<<transform<<std::endl;
  util::PrintTransform(transform);

  // Reset the initial guess flag so that a new transform can be provided (or absolute odom pose to compute a transform from)
  has_initial_guess_ = false;
  
  if (transform_publisher_ != nullptr) {
    transform_publisher_->publish(MatrixToMultiArray(transform));
  }

  if (trans_scan_a_publisher_ != nullptr) {
    trans_scan_a_publisher_->publish(MatrixToPointCloud2(icp_.GetTransformedScanA(), frame_id_));
  }
}
