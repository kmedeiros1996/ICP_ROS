// std
#include <vector>
#include <numeric> // std::accumulate

// Third party
#include <Eigen/Dense>

// ICP
#include "icp_cpp/icp.h"
#include "icp_cpp/icp_utils.h"

ICP::ICP(int max_iterations,  float tolerance) : max_iters_(max_iterations), tolerance_(tolerance), initial_guess_(Eigen::Matrix4d::Identity()), indices_(), dists_(), num_pts_a_(0), num_pts_b_(0) {}

void ICP::CalcClosestNeighborsKDTree(const Eigen::MatrixXd &scan_a) {
  const size_t num_results = 1;
  Eigen::Vector3d src_pt;

  for (int i = 0; i < scan_a.rows(); i++) {
    std::vector<size_t> temp_index(1);
    std::vector<double> temp_dist(1);

    src_pt = scan_a.row(i);
    nanoflann::KNNResultSet<double> results(num_results);
    results.init(&temp_index[0], &temp_dist[0]);

    std::vector<double> query_pt{src_pt(0), src_pt(1), src_pt(2)};
    pc_b_index_->index->findNeighbors(results, &query_pt[0], nanoflann::SearchParams(10));

    indices_.push_back(int(temp_index[0]));
    dists_.push_back(temp_dist[0]);
  }
}

void ICP::SetScanA(const Eigen::MatrixXd &scan_a) {
  input_scan_a_ = scan_a;
  num_pts_a_ = scan_a.rows();
  curr_aligned_scan_a_ = Eigen::MatrixXd::Ones(kDimension, num_pts_a_);
  scan_a_hg_ = Eigen::MatrixXd::Ones(kDimension+1, num_pts_a_);

  // Transform homogeneous pointcloud by initial guess
  for (int pt_index = 0; pt_index < num_pts_a_; pt_index++) {
    Eigen::Vector3d pt = scan_a.row(pt_index);
    Eigen::Vector4d pt_hg = {pt[0], pt[1], pt[2], 1.0};
    Eigen::Vector4d transformed_pt = initial_guess_ * pt_hg;

    scan_a_hg_.col(pt_index) = transformed_pt;
    curr_aligned_scan_a_.col(pt_index) = transformed_pt.head<3>();
  }
}

void ICP::SetScanB(const Eigen::MatrixXd &scan_b) {
  scan_b_ = scan_b;
  num_pts_b_ = scan_b_.rows();
  scan_b_hg_ = Eigen::MatrixXd::Ones(kDimension+1, num_pts_b_);

  pc_b_index_ = std::make_unique<NearestNeighborKDTree>(kDimension, std::cref(scan_b_), 10);
  pc_b_index_->index->buildIndex();

  for (int pt_index = 0; pt_index < num_pts_b_; pt_index++) {
    scan_b_hg_.col(pt_index).head<3>() = scan_b_.row(pt_index);
  }
}

Eigen::Matrix4d ICP::ComputeFinalTransform() {

  transform_matr_ = util::BestFitTransform(input_scan_a_, curr_aligned_scan_a_.transpose());
  return transform_matr_;
}

void ICP::MatchScanAToScanB(double tolerance) {
  double prev_error = 0.0;
  double mean_error = 0.0;
  double diff;

  for (iters_ = 1; iters_ <= max_iters_; iters_++) {
    mean_error = RunOneIteration();

    diff = fabs(mean_error - prev_error);
    if (diff < tolerance_) {
      break;
    }
    prev_error = mean_error;
  }
  ComputeFinalTransform();
}

double ICP::RunOneIteration() {
  indices_.clear();
  dists_.clear();

  CalcClosestNeighborsKDTree(curr_aligned_scan_a_.transpose());

  Eigen::MatrixXd closest_pts_in_b = Eigen::MatrixXd::Ones(kDimension, num_pts_a_);

  for (int pt_index = 0; pt_index < num_pts_a_; pt_index++) {
      closest_pts_in_b.col(pt_index) = scan_b_hg_.col(indices_[pt_index]).head<3>();
  }

  transform_matr_ = util::BestFitTransform(curr_aligned_scan_a_.transpose(), closest_pts_in_b.transpose());

  scan_a_hg_ = transform_matr_ * scan_a_hg_;

  for (int pt_index = 0; pt_index < num_pts_a_; pt_index++) {
    curr_aligned_scan_a_.col(pt_index) = scan_a_hg_.col(pt_index).head<3>();
  }

  return std::accumulate(dists_.begin(), dists_.end(), 0.0) / dists_.size();
}
