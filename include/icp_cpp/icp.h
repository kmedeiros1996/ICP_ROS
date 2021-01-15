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
  * @brief constructor which takes max number of iterations and error convergence tolerance
  */
  ICP(int max_iterations, float tolerance=kLeastSquaresErrorTolerance);

  /*
  * @brief Match the input scan A against the already built KD-Tree from Scan B.
  * Assumes Scan A has already been set via a call to "SetScanA".
  * Stores the homogeneous transform between the original input and the post-alignment input in transform_matr_
  * @param tolerance error diff threshold to consider converged
  */
  void MatchScanAToScanB(double tolerance = kLeastSquaresErrorTolerance);

  /*
  * @brief Set Scan A, populate the scan A homogeneous coordinates matrix (dim+1, n_points), and transform via the initial guess.
  * @param scan_a (n_points * dim) matrix encoding pointcloud A xyz data
  */
  void SetScanA(const Eigen::MatrixXd &scan_a);

  /*
  * @brief Set Scan B, populate the scan B homogeneous coordinates matrix (dim+1, n_points),
  * and build a KD-Tree for Scan A to lookup nearest neighbors to.
  * @param scan_b (n_points * dim) matrix encoding pointcloud B xyz data
  */
  void SetScanB(const Eigen::MatrixXd &scan_b);

  /*
  * @brief runs one iteration of ICP by calculating the closest neighbors in B to each point in A,
  * computing the best fit transform between the closest points in B and the current transformed scan A,
  * transforming A via the computed transformation matrix, and computing the mean error from all the point distances.
  *
  * @return average of all point distances
  */
  double RunOneIteration();

  /*
  * @brief Set initial guess of the transform which maps A and B.
  * @param initial_guess 4D matrix to set initial guess to.
  */
  void SetInitialGuess(const Eigen::Matrix4d &initial_guess) { initial_guess_ = initial_guess; }

  /*-----------------Getters to extract final ICP results*--------------------/
  /*
  * @brief get 4d transformation matrix encoding rotation / translation
  */
  Eigen::Matrix4d GetTransform() {return transform_matr_;}

  /*
  * @brief computes and returns transform_matr_ as the transform between the initial scan A and the current aligned scan A
  */
  Eigen::Matrix4d ComputeFinalTransform();
  /*
  * @brief get 4d transformation matrix encoding rotation / translation
  */
  std::vector<double> GetPrevDistances(){return dists_;}

  /*
  * @brief get the number of iterations it took for ICP to converge
  */
  int GetPrevIterations() const {return iters_;}

  /*
  * @brief get the max number of allowed iterations
  */
  int GetMaxIterations() const {return max_iters_; }

  /*
  * @brief get (n_points * dim) matrix containing aligned pointcloud xyz data
  */
  Eigen::MatrixXd GetTransformedScanA(){return curr_aligned_scan_a_.transpose();}

  /*
  * @brief get (n_points * dim) matrix containing initial input scan
  */
  Eigen::MatrixXd GetInputScanA() const {return input_scan_a_; }

  /*
  * @brief get (n_points * dim) matrix containing scan B
  */
  Eigen::MatrixXd GetScanB() const {return scan_b_; }

  /*
  * @return convergence tolerance for ICP
  */
  float GetTolerance() const { return tolerance_; }

  /*
  * @brief set tolerance for ICP convergence
  */
  void SetTolerance(const float tol) {tolerance_ = tol; }

private:

  /*
  * @brief calculate closest neighbors in Pointcloud B to each point in scan_a.
  * stores results in indices_ and dists_
  */
  void CalcClosestNeighborsKDTree(const Eigen::MatrixXd &scan_a);

  int max_iters_{0};                                  // Max allowed iterations
  int iters_{0};                                      // current iterations
  float tolerance_{kLeastSquaresErrorTolerance};      // ICP convergence tolerance
  std::unique_ptr<NearestNeighborKDTree> pc_b_index_; // Nearest-Neighbor KD-Tree from Scan B (n_points * dim)
  Eigen::Matrix4d transform_matr_;                    // Homogeneous transformation matrix between input scan A and post-alignment scan A
  Eigen::Matrix4d initial_guess_;                     // 4D matrix of initial guess for the transform between A and B
  std::vector<int> indices_;                          // vector of indices to closest points
  std::vector<double> dists_;                         // vector of distances to closest points
  int num_pts_a_;                                     // Number of points in Scan A
  int num_pts_b_;                                     // Number of points in Scan B
  Eigen::MatrixXd input_scan_a_;                      // Input scan A (n_points * dim)
  Eigen::MatrixXd scan_b_;                            // Input scan B (n_points * dim)
  Eigen::MatrixXd curr_aligned_scan_a_;               // Input scan A with current transformation applied (dim * n_points)
  Eigen::MatrixXd scan_a_hg_;                         // Input scan A in homogeneous coordinates (dim+1 * n_points)
  Eigen::MatrixXd scan_b_hg_;                         // Input scan B in homogeneous coordinates (dim+1 * n_points)
};


#endif //ICP_H
