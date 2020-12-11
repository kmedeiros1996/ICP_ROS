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
input_b_topic_(options.input_scan_b_topic) {
  std::cout<<"Starting ICP driver in "<<options.mode<<" mode"<<std::endl;
  mode_ = (options.mode == "sequential") ? MODE_SEQUENTIAL : MODE_A_TO_B;

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

void ScanMatchDriver::ProcessScanSequential(const Eigen::MatrixXd& pc_matrix) {
  scan_a_ = pc_matrix;
  if (has_both_scans_) {
    std::cout<<"Waiting for ICP to converge...";
    icp_.MatchScanAToScanB(scan_a_);
    std::cout<<" Done! "<<std::endl;
    Eigen::Matrix4d transform = icp_.GetTransform();
    std::cout<<"Transform found after "<<icp_.GetPrevIterations()<< " iterations: \n\n"<<transform<<std::endl;

    transformed_scan_a_ = icp_.GetAlignedScan();
    scan_b_ = transformed_scan_a_;
    std::cout<<"Setting Scan B and building kd tree...";
    icp_.SetScanB(scan_b_);
    std::cout<<" Done! "<<std::endl;

    if (transform_publisher_ != nullptr) {
      transform_publisher_->publish(MatrixToMultiArray(transform));
    }

    if (scan_a_publisher_ != nullptr) {
      scan_a_publisher_->publish(MatrixToPointCloud2(scan_a_, frame_id_));
    }

    if (trans_scan_a_publisher_ != nullptr) {
      trans_scan_a_publisher_->publish(MatrixToPointCloud2(transformed_scan_a_, frame_id_));
    }

    if (scan_b_publisher_!= nullptr) {
      scan_b_publisher_->publish(MatrixToPointCloud2(scan_b_, frame_id_));
    }

  } else {
    std::cout<<"Sequential Scan Match: initializing first scan..."<<std::endl;
    has_both_scans_ = true;
    scan_b_ = scan_a_;
    icp_.SetScanB(scan_b_);
    if (scan_b_publisher_!= nullptr) {
      scan_b_publisher_->publish(MatrixToPointCloud2(scan_b_, frame_id_));
    }
  }
}

void ScanMatchDriver::ProcessScanA(const Eigen::MatrixXd& pc_matrix) {
  scan_a_ = pc_matrix;

  if (has_both_scans_) {
    icp_.MatchScanAToScanB(scan_a_);

    Eigen::Matrix4d transform = icp_.GetTransform();
    std::cout<<"Transform found after "<<icp_.GetPrevIterations()<< " iterations: \n\n"<<transform<<std::endl;

    transformed_scan_a_ = icp_.GetAlignedScan();

    if (transform_publisher_ != nullptr) {
      transform_publisher_->publish(MatrixToMultiArray(transform));
    }

    if (scan_a_publisher_ != nullptr) {
      scan_a_publisher_->publish(MatrixToPointCloud2(scan_a_, frame_id_));
    }

    if (trans_scan_a_publisher_ != nullptr) {
      trans_scan_a_publisher_->publish(MatrixToPointCloud2(transformed_scan_a_, frame_id_));
    }

  } else {
    std::cout<<"A to B Scan Match: Scan A has been overridden. Awaiting scan B to match against..."<<std::endl;
  }
}

void ScanMatchDriver::ProcessScanB(const Eigen::MatrixXd& pc_matrix) {
  scan_b_ = pc_matrix;
  icp_.SetScanB(scan_b_);
  std::cout<<"A to B Scan Match: B is set"<<std::endl;
  has_both_scans_ = true;

  if (scan_b_publisher_!= nullptr) {
    scan_b_publisher_->publish(MatrixToPointCloud2(scan_b_, frame_id_));
  }
}
