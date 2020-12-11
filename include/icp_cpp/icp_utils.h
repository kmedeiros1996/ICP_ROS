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
  * @return transform between pointcloud_a and pointcloud_b
  */
  Eigen::MatrixXd BestFitTransform(const Eigen::MatrixXd &scan_a, const Eigen::MatrixXd &scan_b);

  /*
  * @brief print out matrix dims in (row, col) format.
  */
  void PrintMatDims(const Eigen::MatrixXd& mat);
}

#endif //ICP_UTILS_H
