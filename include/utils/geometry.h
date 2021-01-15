#ifndef ICP_GEOMETRY_H
#define ICP_GEOMETRY_H

// Third Party
#include <Eigen/Dense>

namespace util {

  /*
  * @brief generate a 4D transformation matrix from x, y, z coordinates and orientation encoded in r, p, y
  */
  Eigen::Matrix4d GenTransformMatrix(double x, double y, double z, double roll, double pitch, double yaw);

  /*
  * @brief generate a 4D transformation matrix from a vector taking the form of <x, y, z, r, p, y>
  * @param pv pose_vector containing <x,y,z,r,p,y> information
  */
  Eigen::Matrix4d GenTransformMatrix(const Eigen::VectorXd& pv);

  /*
  * @brief Generate a pose vector of form <x, y, z, r, p, y> from a 4D affine transformation matrix.
  */
  Eigen::VectorXd GenPoseVector(const Eigen::Matrix4d& transform);

  /*
  * @brief transform a point via a 4D transformation matrix.
  * @param point a (dim * 1) 3D point
  * @param transform transformation matrix
  * @return transformed point
  */
  Eigen::Vector3d TransformPoint(const Eigen::Vector3d& point, const Eigen::Matrix4d& transformation_matrix);

  /*
  * @brief transform a pointcloud via a 4D transformation matrix.
  * @param cloud a (dim * n_pts) matrix of 3D points
  * @param transform transformation matrix
  * @return transformed point
  */
  Eigen::MatrixXd TransformCloud(const Eigen::MatrixXd& cloud, const Eigen::Matrix4d& transformation_matrix);

} // namespace util

#endif //ICP_GEOMETRY_H
