/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#ifndef BRICS_3D_ROSNORMAL3D_H_
#define BRICS_3D_ROSNORMAL3D_H_
#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/NormalSet3D.h"
#include "ROSfeature.h"

//ToDo Replace using concrete classes in algoritm/featureExtraction

namespace brics_3d{


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
                      Eigen::Vector4d &plane_parameters, double &curvature)
{
  if (cloud->getSize()== 0)
  {
    plane_parameters.setConstant (std::numeric_limits<double>::quiet_NaN ());
    curvature = std::numeric_limits<double>::quiet_NaN ();
    return;
  }

  // Placeholder for the 3x3 covariance matrix at each surface patch
    EIGEN_ALIGN_MEMORY Eigen::Matrix3d covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4d xyz_centroid;

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
  computePointNormal (PointCloud3D* cloud, const std::vector<int> *indices,
                      Eigen::Vector4d &plane_parameters, double &curvature)
{
  if (indices->size () == 0)
  {
    plane_parameters.setConstant (std::numeric_limits<double>::quiet_NaN ());
    curvature = std::numeric_limits<double>::quiet_NaN ();
    return;
  }

  // Placeholder for the 3x3 covariance matrix at each surface patch
  EIGEN_ALIGN_MEMORY Eigen::Matrix3d covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4d xyz_centroid;

  // Estimate the XYZ centroid
  compute3DCentroid (cloud, *indices, xyz_centroid);

  // Compute the 3x3 covariance matrix
  computeCovarianceMatrix (cloud, *indices, xyz_centroid, covariance_matrix);

  // Get the plane normal and surface curvature
  solvePlaneParameters (covariance_matrix, xyz_centroid, plane_parameters, curvature);
}



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
                      Eigen::Vector4d &plane_parameters, double &curvature)
{
  if (indices.size () == 0)
  {
    plane_parameters.setConstant (std::numeric_limits<double>::quiet_NaN ());
    curvature = std::numeric_limits<double>::quiet_NaN ();
    return;
  }

  // Placeholder for the 3x3 covariance matrix at each surface patch
	EIGEN_ALIGN_MEMORY Eigen::Matrix3d covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4d xyz_centroid;

  // Estimate the XYZ centroid
  compute3DCentroid (cloud, indices, xyz_centroid);

  // Compute the 3x3 covariance matrix
  computeCovarianceMatrix (cloud, indices, xyz_centroid, covariance_matrix);

  // Get the plane normal and surface curvature
  solvePlaneParameters (covariance_matrix, xyz_centroid, plane_parameters, curvature);
}


/** \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
  * and return the estimated plane parameters together with the surface curvature.
  * \param cloud the input point cloud
  * \param indices the point cloud indices that need to be used
  * \param nx the resultant X component of the plane normal
  * \param ny the resultant Y component of the plane normal
  * \param nz the resultant Z component of the plane normal
  * \param curvature the estimated surface curvature as a measure of
  * \f[
  * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
  * \f]
  */
inline void
  computePointNormal (PointCloud3D *cloud, const std::vector<int> &indices,
                      double &nx, double &ny, double &nz, double &curvature)
{
  if (indices.size () == 0)
  {
    nx = ny = nz = curvature = std::numeric_limits<double>::quiet_NaN ();
    return;
  }

  // Placeholder for the 3x3 covariance matrix at each surface patch
	EIGEN_ALIGN_MEMORY Eigen::Matrix3d covariance_matrix;
     // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
     Eigen::Vector4d xyz_centroid;


  // Estimate the XYZ centroid
  compute3DCentroid (cloud, indices, xyz_centroid);

  // Compute the 3x3 covariance matrix
  computeCovarianceMatrix (cloud, indices, xyz_centroid, covariance_matrix);

  // Get the plane normal and surface curvature
  solvePlaneParameters (covariance_matrix, nx, ny, nz, curvature);
}

}

#endif /* BRICS_3D_ROSNORMAL3D_H_ */
