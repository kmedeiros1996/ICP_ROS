// Third Party
#include <Eigen/Dense>

// ICP
#include <utils/geometry.h>

Eigen::Matrix4d util::GenTransformMatrix(double x, double y, double z, double roll, double pitch, double yaw) {

  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                  * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

  transformation_matrix.block<3,3>(0,0) = rotation_matrix;

  Eigen::Vector3d translation_vec (x, y, z);
  transformation_matrix.block<3,1>(0,3) = translation_vec;

  return transformation_matrix;
}

Eigen::Matrix4d util::GenTransformMatrix(const Eigen::VectorXd& pv) {

  assert(pv.size() == 6); // This method expects a 6D pose vector of form <x,y,z,r,p,y>
  return GenTransformMatrix(pv[0], pv[1], pv[2], pv[3], pv[4], pv[5]);
}

Eigen::VectorXd util::GenPoseVector(const Eigen::Matrix4d& transform) {
  Eigen::VectorXd pose_vec(6);

  Eigen::Matrix3d rotation_matrix = transform.block<3,3>(0,0);
  Eigen::Vector3d translation_vector = transform.block<3,1>(0,3);
  Eigen::Vector3d euler = rotation_matrix.eulerAngles(0,1,2);

  pose_vec.head<3>() = translation_vector;
  pose_vec.tail<3>() = euler;
  return pose_vec;
}

Eigen::Vector3d util::TransformPoint(const Eigen::Vector3d& point, const Eigen::Matrix4d& transformation_matrix) {

  Eigen::Vector4d point_hg;
  point_hg << point[0],point[1], point[2], 1;

  return (transformation_matrix * point_hg).head<3>();
}

Eigen::MatrixXd util::TransformCloud(const Eigen::MatrixXd& cloud, const Eigen::Matrix4d& transformation_matrix) {
  Eigen::MatrixXd out = Eigen::MatrixXd::Zero(cloud.rows(), cloud.cols());

  Eigen::Matrix3d rotation_matrix = transformation_matrix.topLeftCorner<3,3>();
  Eigen::Vector3d translation_vec = transformation_matrix.topRightCorner<3,1>();

  for (int pt_index = 0; pt_index < cloud.rows(); pt_index++) {
    out.row(pt_index) = cloud.row(pt_index)*rotation_matrix + translation_vec.transpose();
  }

  return out;
}
