#ifndef ICP_H
#define ICP_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include "nanoflann/nanoflann.hpp"

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> nn_kd_tree;


class ICP
{

private:
  int max_iters;
  Eigen::Matrix4d transform_matr;
  std::vector<int> indices;
  std::vector<double> dists;
  int iters;
  Eigen::MatrixXd src_3d;



public:

  ICP(int m_it);
  Eigen::MatrixXd best_fit_transform(const Eigen::MatrixXd &pointcloud_A, const Eigen::MatrixXd &pointcloud_B);
  float euc_dist(const Eigen::Vector3d &pt_a, const Eigen::Vector3d &pt_b);
  void calc_closest_neighbors_kdtree(const Eigen::MatrixXd &src, const nn_kd_tree &dst_tree);
  void run_scan_matcher(const Eigen::MatrixXd &pointcloud_A, const Eigen::MatrixXd &pointcloud_B, double tolerance = 0.00001);

  //Extract final ICP results
  Eigen::Matrix4d get_transformation() {return transform_matr;}
  std::vector<double> get_last_distances(){return dists;}
  int get_last_iterations(){return iters;}
  Eigen::MatrixXd get_transformed_pointcloud(){return src_3d;}
};


#endif //ICP_H
