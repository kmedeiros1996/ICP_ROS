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
  std::cout<<"Starting ICP driver in "<<mode_<<" mode"<<std::endl;

  if (show_each_step_) {
    std::cout<<"Running in debug SHOW_EACH_STEP mode. "<<std::endl;
    std::cout<<"Scans will be published after every iteration every "<<options.stepwise_time_interval<<"secs"<<std::endl;
    stepwise_time_interval_ = std::make_unique<ros::Duration>(options.stepwise_time_interval);
  }

  InitializeSubscribers(options);
  InitializePublishers(options);
  ros::spin();
}

void ScanMatchDriver::InitializePublishers(const ProgramOptions& options) {

    transform_publisher_ = std::make_unique<ros::Publisher>(node_handle_.advertise<std_msgs::Float64MultiArray>(options.output_transform_matrix_topic, 1000));
    std::cout<<"Initialized Transformation Matrix publisher on topic "<<options.output_transform_matrix_topic<<std::endl;

    if (options.pub_scan_a) {
      scan_a_publisher_ = std::make_unique<ros::Publisher>(node_handle_.advertise<sensor_msgs::PointCloud2>(options.output_scan_a_topic, 1000));
      std::cout<<"Initialized Scan A publisher on topic "<<options.output_scan_a_topic<<std::endl;
    }
    if (options.pub_trans_scan_a) {
      trans_scan_a_publisher_ = std::make_unique<ros::Publisher>(node_handle_.advertise<sensor_msgs::PointCloud2>(options.output_trans_scan_a_topic, 1000));
      std::cout<<"Initialized Transformed Scan A publisher on topic "<<options.output_trans_scan_a_topic<<std::endl;
    }
    if (options.pub_scan_b) {
      scan_b_publisher_ = std::make_unique<ros::Publisher>(node_handle_.advertise<sensor_msgs::PointCloud2>(options.output_scan_b_topic, 1000));
      std::cout<<"Initialized Scan B publisher on topic "<<options.output_scan_b_topic<<std::endl;
    }

    if (options.show_each_step) {
      step_scan_a_publisher_ = std::make_unique<ros::Publisher>(node_handle_.advertise<sensor_msgs::PointCloud2>(options.output_stepwise_a_topic, 1000));
      std::cout<<"Initialized Stepwise Scan A publisher on topic "<<options.output_stepwise_a_topic<<std::endl;
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
    } else {
      throw std::runtime_error("Invalid input scan message type!");
    }
}

void ScanMatchDriver::InitializeSubscribers(const ProgramOptions& options) {
    scan_a_subscriber_ = MakeSubscriber(input_a_topic_, options.input_scan_a_type, 1000);
    std::cout<<"Receiving input scan A of type "<< options.input_scan_a_type<<" on topic "<<options.input_scan_a_topic<<std::endl;

    guess_subscriber_ = std::make_unique<ros::Subscriber>(node_handle_.subscribe( options.initial_guess_topic, 1000, &ScanMatchDriver::InitialGuessCallback, this));

    if (mode_ == MODE_A_TO_B) {
    scan_b_subscriber_ = MakeSubscriber(input_b_topic_, options.input_scan_b_type, 1000);
      std::cout<<"Receiving input scan B of type "<< options.input_scan_b_type<<" on topic "<<options.input_scan_b_topic<<std::endl;
    }
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

void ScanMatchDriver::InitialGuessCallback(const std_msgs::Float64MultiArray& initial_guess) {
  icp_.SetInitialGuess(MultiArrayToMatrix(initial_guess));
}

void ScanMatchDriver::ProcessScanSequential(const Eigen::MatrixXd& pc_matrix) {
  scan_a_ = pc_matrix;

  if (scan_a_publisher_ != nullptr) {
    scan_a_publisher_->publish(MatrixToPointCloud2(scan_a_, frame_id_));
  }

  if (has_both_scans_) {
    if (show_each_step_) {
      RunICPStepwiseMode();
    } else {
      RunICPRegularMode();
    }

  } else {
    std::cout<<"Sequential Scan Match: initializing first scan..."<<std::endl;
    ProcessScanB(scan_a_);
  }
}

void ScanMatchDriver::ProcessScanA(const Eigen::MatrixXd& pc_matrix) {
  scan_a_ = pc_matrix;

  if (scan_a_publisher_ != nullptr) {
    scan_a_publisher_->publish(MatrixToPointCloud2(scan_a_, frame_id_));
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
  scan_b_ = pc_matrix;
  icp_.SetScanB(scan_b_);
  std::cout<<"Done!"<<std::endl;
  has_both_scans_ = true;

  if (scan_b_publisher_!= nullptr) {
    scan_b_publisher_->publish(MatrixToPointCloud2(scan_b_, frame_id_));
  }
}

void ScanMatchDriver::RunICPRegularMode() {
  std::cout<<"Waiting for ICP to converge...";
  icp_.MatchScanAToScanB(scan_a_);
  std::cout<<"Done!"<<std::endl;

  Eigen::Matrix4d transform = icp_.GetTransform();
  std::cout<<"Transform found after "<<icp_.GetPrevIterations()<< " iterations: \n\n"<<transform<<std::endl;

  transformed_scan_a_ = icp_.GetAlignedScan();

  if (transform_publisher_ != nullptr) {
    transform_publisher_->publish(MatrixToMultiArray(transform));
  }

  if (trans_scan_a_publisher_ != nullptr) {
    trans_scan_a_publisher_->publish(MatrixToPointCloud2(transformed_scan_a_, frame_id_));
  }
}

void ScanMatchDriver::RunICPStepwiseMode() {
  std::cout<<"Running scan match in stepwise mode."<<std::endl;
  icp_.SetScanA(scan_a_);

  double prev_error = 0.0;
  double mean_error = 0.0;
  double diff;
  int iters = 0;

  for (iters = 0; iters <= icp_.GetMaxIterations(); iters++) {
    mean_error = icp_.RunOneIteration();
    transformed_scan_a_ = icp_.GetAlignedScan();

    if (step_scan_a_publisher_ != nullptr) {
      step_scan_a_publisher_->publish(MatrixToPointCloud2(transformed_scan_a_, frame_id_));
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
  std::cout<<"Transform found after "<<iters<< " iterations: \n\n"<<transform<<std::endl;

  if (transform_publisher_ != nullptr) {
    transform_publisher_->publish(MatrixToMultiArray(transform));
  }

  if (trans_scan_a_publisher_ != nullptr) {
    trans_scan_a_publisher_->publish(MatrixToPointCloud2(transformed_scan_a_, frame_id_));
  }
}
