/*
 * ROSnormal3D.h
 *
 *  Created on: Apr 18, 2011
 *      Author: reon
 */

#ifndef ROSNORMAL3D_H_
#define ROSNORMAL3D_H_
#include "core/PointCloud3D.h"
#include "ROSfeature.h"

namespace BRICS_3D{
/** \brief Compute the Least-Squares plane fit for a given set of points, and return the estimated plane
  * parameters together with the surface curvature.
  * \param cloud the input point cloud
  * \param plane_parameters the plane parameters as: a, b, c, d (ax + by + cz + d = 0)
  * \param curvature the estimated surface curvature as a measure of
  * \f[
  * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
  * \f]
  */
template <typename PointT> inline void
  computePointNormal (PointCloud3D* cloud,
                      Eigen::Vector4f &plane_parameters, float &curvature)
{
  if (cloud->getSize()== 0)
  {
    plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
    curvature = std::numeric_limits<float>::quiet_NaN ();
    return;
  }
  // Placeholder for the 3x3 covariance matrix at each surface patch
  Eigen::Matrix3f covariance_matrix;
  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid
  compute3DCentroid (cloud, xyz_centroid);

  // Compute the 3x3 covariance matrix
  computeCovarianceMatrix (cloud, xyz_centroid, covariance_matrix);

  // Get the plane normal and surface curvature
  solvePlaneParameters (covariance_matrix, xyz_centroid, plane_parameters, curvature);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
  * and return the estimated plane parameters together with the surface curvature.
  * \param cloud the input point cloud
  * \param indices the point cloud indices that need to be used
  * \param plane_parameters the plane parameters as: a, b, c, d (ax + by + cz + d = 0)
  * \param curvature the estimated surface curvature as a measure of
  * \f[
  * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
  * \f]
  */
inline void
  computePointNormal (PointCloud3D* cloud, const std::vector<int> &indices,
                      Eigen::Vector4f &plane_parameters, float &curvature)
{
  if (indices.size () == 0)
  {
    plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
    curvature = std::numeric_limits<float>::quiet_NaN ();
    return;
  }
  // Placeholder for the 3x3 covariance matrix at each surface patch
  EIGEN_ALIGN_128 Eigen::Matrix3f covariance_matrix;
  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid
  compute3DCentroid (cloud, indices, xyz_centroid);

  // Compute the 3x3 covariance matrix
  computeCovarianceMatrix (cloud, indices, xyz_centroid, covariance_matrix);

  // Get the plane normal and surface curvature
  solvePlaneParameters (covariance_matrix, xyz_centroid, plane_parameters, curvature);
}


}

#endif /* ROSNORMAL3D_H_ */
