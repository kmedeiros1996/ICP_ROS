#ifndef ICP_UTILS_H
#define ICP_UTILS_H

// std
#include <iostream>

// Third Party
#include <Eigen/Dense>
#include "nanoflann/nanoflann.hpp"

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> NearestNeighborKDTree;

namespace util {
  
  /*
  * @brief compute the centroid of a pointcloud by taking the mean of all points
  * @param pointcloud pointcloud of size (dim * n_pts)
  * @return centroid of the pointcloud
  */
  Eigen::Vector3d ComputeCentroid(const Eigen::MatrixXd& pointcloud);

  /*
  * @brief compute the rotation matrix between two centered pointclouds via singular value decomposition.
  * @return matrix encoding 3d rotation between shifted_pc_a and shifted_pc_b
  */
  Eigen::MatrixXd ComputeRotationMatrix(Eigen::MatrixXd& shifted_pc_a, Eigen::MatrixXd& shifted_pc_b);

  /*
  * @brief compute the 4d matrix encoding rotation + translation between two pointclouds
  * @param scan_a pointcloud data matrix of size (dim * n_points)
  * @param scan_b pointcloud data matrix of size (dim * n_points)
  * @return transform between pointcloud_a and pointcloud_b
  */
  Eigen::MatrixXd BestFitTransform(const Eigen::MatrixXd &scan_a, const Eigen::MatrixXd &scan_b);

  /*
  * @brief print out matrix dims in (row, col) format.
  */
  void PrintMatDims(const Eigen::MatrixXd& mat);

  /*
  * @brief print the xyz/rpy encoded in a 4D transformation matrix.
  */
  void PrintTransform(const Eigen::Matrix4d& transform);
} // namespace util

#endif //ICP_UTILS_H
