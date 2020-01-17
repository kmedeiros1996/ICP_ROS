#ifndef ICP_H
#define ICP_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include "nanoflann/nanoflann.hpp"

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> nn_kd_tree;

class ICP {
public:
  ICP(int m_it);
  Eigen::MatrixXd BestFitTransform(const Eigen::MatrixXd &pointcloud_A, const Eigen::MatrixXd &pointcloud_B);
  double EuclideanDistance(const Eigen::Vector3d &pt_a, const Eigen::Vector3d &pt_b);
  void CalcClosestNeighborsKDTree(const Eigen::MatrixXd &src, const nn_kd_tree &dst_tree);
  void Run(const Eigen::MatrixXd &pointcloud_A, const Eigen::MatrixXd &pointcloud_B, double tolerance = 0.00001);

  //Extract final ICP results
  Eigen::Matrix4d GetTransform() {return transform_matr_;}
  std::vector<double> GetPrevDistances(){return dists_;}
  int GetPrevIterations(){return iters_;}
  Eigen::MatrixXd GetTransformedPointcloud(){return src_3d_;}

private:
  int max_iters_{0};
  int iters_{0};
  Eigen::Matrix4d transform_matr_;
  std::vector<int> indices_;
  std::vector<double> dists_;
  Eigen::MatrixXd src_3d_;
};


#endif //ICP_H
