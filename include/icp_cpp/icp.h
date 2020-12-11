#ifndef ICP_H
#define ICP_H

// std
#include <vector>
#include <iostream>
#include <memory>

// Third Party
#include <Eigen/Dense>

// ICP
#include "icp_cpp/icp_utils.h"

// Dimension of input pointcloud data
constexpr int kDimension = 3;

// Default error diff threshold for ICP to be considered converged
constexpr float kLeastSquaresErrorTolerance = 0.00001;

/*
* ICP Scan Matcher class.
*/
class ICP {
public:

  /*
  * @brief constructor which takes max number of iterations
  */
  ICP(int max_iterations);

  /*
  * @brief Match an input scan A against the already built KD-Tree from Scan B.
  * Stores the homogeneous transform between the original input and the post-alignment input in transform_matr_
  * @param scan_a (n_points * dim) matrix encoding pointcloud A xyz data
  * @param tolerance error diff threshold to consider converged
  */
  void MatchScanAToScanB(const Eigen::MatrixXd &scan_a, double tolerance = kLeastSquaresErrorTolerance);

  /*
  * @brief Set Scan B and build a KD-Tree for Scan A to match against
  * @param scan_b (n_points * dim) matrix encoding pointcloud B xyz data
  */
  void SetScanB(const Eigen::MatrixXd &scan_b);

  /*-----------------Getters to extract final ICP results*--------------------/
  /*
  * @brief get 4d transformation matrix encoding rotation / translation
  */
  Eigen::Matrix4d GetTransform() {return transform_matr_;}

  /*
  * @brief get 4d transformation matrix encoding rotation / translation
  */
  std::vector<double> GetPrevDistances(){return dists_;}

  /*
  * @brief get the number of iterations it took for ICP to converge
  */
  int GetPrevIterations(){return iters_;}

  /*
  * @brief get (n_points * dim) matrix containing aligned pointcloud xyz data
  */
  Eigen::MatrixXd GetAlignedScan(){return curr_aligned_scan_a_;}

private:

  /*
  * @brief calculate closest neighbors in Pointcloud B to each point in scan_a.
  * stores results in indices_ and dists_
  */
  void CalcClosestNeighborsKDTree(const Eigen::MatrixXd &scan_a);

  int max_iters_{0};                                  // Max allowed iterations
  int iters_{0};                                      // current iterations
  std::unique_ptr<NearestNeighborKDTree> pc_b_index_; // Nearest-Neighbor KD-Tree from Scan B (n_points * dim)
  Eigen::Matrix4d transform_matr_;                    // Homogeneous transformation matrix between input scan A and post-alignment scan A
  std::vector<int> indices_;                          // vector of indices to closest points
  std::vector<double> dists_;                         // vector of distances to closest points
  Eigen::MatrixXd curr_aligned_scan_a_;               // Input scan A with current transformation applied (n_points * dim)
  Eigen::MatrixXd scan_a_hg_;                         // Input scan A in homogeneous coordinates (n_points * dim+1)
  Eigen::MatrixXd scan_b_hg_;                         // Input scan B in homogeneous coordinates (n_points * dim+1)
};


#endif //ICP_H
