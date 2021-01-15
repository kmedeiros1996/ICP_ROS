// Third Party
#include <Eigen/Dense>

// ICP
#include "icp_cpp/icp_utils.h"

Eigen::Vector3d util::ComputeCentroid(const Eigen::MatrixXd& pointcloud) {
  Eigen::Vector3d centroid (0.,0.,0.);
  const int num_pts = pointcloud.rows();

  for (int i = 0; i < num_pts; i++) {
    centroid += pointcloud.row(i).transpose();
  }

  centroid/=num_pts;
  return centroid;
}

Eigen::MatrixXd util::ComputeRotationMatrix(Eigen::MatrixXd& shifted_pc_a, Eigen::MatrixXd& shifted_pc_b) {
  Eigen::MatrixXd a_transpose_b = shifted_pc_a.transpose() * shifted_pc_b;
  Eigen::MatrixXd rotation_matrix;

  Eigen::JacobiSVD<Eigen::MatrixXd> SVD(a_transpose_b, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::MatrixXd U = SVD.matrixU();
  Eigen::MatrixXd S = SVD.singularValues();
  Eigen::MatrixXd V = SVD.matrixV();
  Eigen::MatrixXd v_transpose = V.transpose();
  Eigen::MatrixXd u_transpose = U.transpose();

  rotation_matrix = V*u_transpose;

  // If the rotation matrix has zero determinant, we need to negate the sin/cos values
  if (rotation_matrix.determinant() < 0) {
    V.block<1,3>(2,0) *=-1;
    rotation_matrix = V*u_transpose;
  }

  return rotation_matrix;
}


Eigen::MatrixXd util::BestFitTransform(const Eigen::MatrixXd &scan_a, const Eigen::MatrixXd &scan_b) {
  Eigen::MatrixXd transformation_matrix = Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd shifted_A = scan_a;
  Eigen::MatrixXd shifted_B = scan_b;
  Eigen::Vector3d centroid_a = ComputeCentroid(scan_a);
  Eigen::Vector3d centroid_b = ComputeCentroid(scan_b);

  const int num_pts_a = scan_a.rows();
  const int num_pts_b = scan_b.rows();

  for(int i = 0; i < num_pts_a; i++) {
    shifted_A.row(i) = scan_a.row(i) - centroid_a.transpose();
  }

  for (int i = 0; i < num_pts_b; i++) {
    shifted_B.row(i) = scan_b.row(i) - centroid_b.transpose();
  }

  Eigen::MatrixXd rotation_matrix = ComputeRotationMatrix(shifted_A, shifted_B);
  Eigen::Vector3d translation_vector = centroid_b - rotation_matrix * centroid_a;

  transformation_matrix.block<3,3>(0,0) = rotation_matrix;
  transformation_matrix.block<3,1>(0,3) = translation_vector;

  return transformation_matrix;
}

void util::PrintMatDims(const Eigen::MatrixXd& mat) {
  std::cout<<"("<<mat.rows()<<", "<<mat.cols()<<")"<<std::endl;
}

void util::PrintTransform(const Eigen::Matrix4d& transform) {
  Eigen::Matrix3d rotation_matrix = transform.block<3,3>(0,0);
  Eigen::Vector3d translation_vector = transform.block<3,1>(0,3);
  Eigen::Vector3d euler = rotation_matrix.eulerAngles(0,1,2);

  std::cout<<"XYZ: "<<translation_vector.transpose()<<std::endl
  <<"RPY: "<<euler.transpose() * 180.0 / M_PI<<std::endl;
}
