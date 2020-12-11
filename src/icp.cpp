// std
#include <vector>
#include <numeric> // std::accumulate

// Third party
#include <Eigen/Dense>

// ICP
#include "icp_cpp/icp.h"
#include "icp_cpp/icp_utils.h"

ICP::ICP(int max_iterations) : max_iters_(max_iterations), indices_(), dists_() {}

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

void ICP::SetScanB(const Eigen::MatrixXd &scan_b) {
  int num_pts_b = scan_b.rows();
  scan_b_hg_ = Eigen::MatrixXd::Ones(num_pts_b, kDimension+1);

  pc_b_index_ = std::make_unique<NearestNeighborKDTree>(kDimension, std::cref(scan_b), 10);
  pc_b_index_->index->buildIndex();

  for (int pt_index = 0; pt_index < num_pts_b; pt_index++) {
    scan_b_hg_.row(pt_index).head<3>() = scan_b.row(pt_index);
  }
}

void ICP::MatchScanAToScanB(const Eigen::MatrixXd &scan_a, double tolerance) {
  int num_pts_a = scan_a.rows();

  curr_aligned_scan_a_ = Eigen::MatrixXd::Ones(num_pts_a, kDimension);
  scan_a_hg_ = Eigen::MatrixXd::Ones(num_pts_a, kDimension+1);


  for (int pt_index = 0; pt_index < num_pts_a; pt_index++) {
    scan_a_hg_.row(pt_index).head<3>() = scan_a.row(pt_index);
    curr_aligned_scan_a_.row(pt_index) = scan_a.row(pt_index);
  }

  double prev_error = 0.0;
  double mean_error = 0.0;
  double diff;

  for (iters_ = 1; iters_ <= max_iters_; iters_++) {
    indices_.clear();
    dists_.clear();

    CalcClosestNeighborsKDTree(curr_aligned_scan_a_);

    Eigen::MatrixXd closest_pts_in_b = Eigen::MatrixXd::Ones(num_pts_a, kDimension);

    for (int pt_index = 0; pt_index < num_pts_a; pt_index++) {
        closest_pts_in_b.row(pt_index) = scan_b_hg_.row(indices_[pt_index]).head<3>();
    }

    transform_matr_ = util::BestFitTransform(curr_aligned_scan_a_, closest_pts_in_b);

    scan_a_hg_ = scan_a_hg_ * transform_matr_;

    for (int pt_index = 0; pt_index < num_pts_a; pt_index++) {
      curr_aligned_scan_a_.row(pt_index) = scan_a_hg_.row(pt_index).head<3>();
    }

    mean_error = std::accumulate(dists_.begin(), dists_.end(), 0.0) / dists_.size();
    diff = mean_error-prev_error;
    diff = (diff < 0) ? diff*-1 : diff;

    if (diff < tolerance){
      break;
    }
    prev_error = mean_error;
    iters_++;
  }
  transform_matr_ = util::BestFitTransform(scan_a, curr_aligned_scan_a_);
}
