/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: feature.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */

#ifndef PCL_FEATURE_H_
#define PCL_FEATURE_H_

#include "core/PointCloud3D.h"
#include "core/Normal3D.h"
#include <Eigen/Dense>
namespace BRICS_3D
{
  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points using their indices and return it as a 3D vector.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the output centroid
    */
inline void
    compute3DCentroid (PointCloud3D *cloud, const std::vector<int> &indices,
                       Eigen::Vector4d &centroid)
  {

	std::vector<Point3D> *points = cloud->getPointCloud();
    // Initialize to 0
    centroid.setZero ();
    if (indices.size () == 0) 
      return;
    // For each point in the cloud
    int cp = 0;
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (isnan ( points->data()[indices[i]].getX()) || isnan (points->data()[indices[i]].getY()) || isnan (points->data()[indices[i]].getZ()))
        continue;

      centroid[0] += points->data()[indices[i]].getX();
      centroid[1] += points->data()[indices[i]].getY();
      centroid[2] += points->data()[indices[i]].getZ();
      cp++;
    }
    centroid /= cp;
  }

  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points and return it as a 3D vector.
    * \param cloud the input point cloud
    * \param centroid the output centroid
    */
 inline void
    compute3DCentroid (PointCloud3D* cloud, Eigen::Vector4d &centroid)
  {
	  std::vector<Point3D> *points = cloud->getPointCloud();
	  // Initialize to 0
    centroid.setZero ();
    if (cloud->getSize() == 0)
      return;
    // For each point in the cloud
    int cp = 0;
    for (size_t i = 0; i < cloud->getSize (); ++i)
    {
      // Check if the point is invalid
      if (isnan (points->data()[i].getX()) || isnan (points->data()[i].getY()) || isnan (points->data()[i].getZ()))
        continue;

      centroid[0] += points->data()[i].getX();
      centroid[1] += points->data()[i].getY();
      centroid[2] += points->data()[i].getZ();
      cp++;
    }
    centroid /= cp;
  }

   /** \brief Compute the 3x3 covariance matrix of a given set of points.
    * The result is returned as a Eigen::Matrix3d.
    * \param cloud the input point cloud
    * \param centroid the centroid of the set of points in the cloud
    * \param covariance_matrix the resultant 3x3 covariance matrix
    */
inline void
    computeCovarianceMatrix (PointCloud3D* cloud,
                             const Eigen::Vector4d &centroid, Eigen::Matrix3d &covariance_matrix)
  {
	  std::vector<Point3D> *points = cloud->getPointCloud();
	  // Initialize to 0
    covariance_matrix.setZero ();

    if (cloud->getSize() == 0)
      return;
    // For each point in the cloud
    for (size_t i = 0; i < cloud->getSize(); ++i)
    {
      // Check if the point is invalid
      if (isnan (points->data()[i].getX()) || isnan (points->data()[i].getY()) || isnan (points->data()[i].getZ()))
        continue;

      Point3D p;
      p.setX(points->data()[i].getX() - centroid[0]);
      p.setY(points->data()[i].getY() - centroid[1]);
      p.setZ(points->data()[i].getZ() - centroid[2]);

      double demean_xy = p.getX() * p.getY();
      double demean_xz = p.getX() * p.getZ();
      double demean_yz = p.getY() * p.getZ();

      covariance_matrix (0, 0) += p.getX() * p.getX();
      covariance_matrix (0, 1) += demean_xy;
      covariance_matrix (0, 2) += demean_xz;
      covariance_matrix (1, 0) += demean_xy;
      covariance_matrix (1, 1) += p.getY() * p.getY();
      covariance_matrix (1, 2) += demean_yz;
      covariance_matrix (2, 0) += demean_xz;
      covariance_matrix (2, 1) += demean_yz;
      covariance_matrix (2, 2) += p.getZ() * p.getZ();
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
    * The result is returned as a Eigen::Matrix3d.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the centroid of the set of points in the cloud
    * \param covariance_matrix the resultant 3x3 covariance matrix
    */
inline void
    computeCovarianceMatrix (PointCloud3D* cloud, const std::vector<int> &indices,
                             const Eigen::Vector4d &centroid, Eigen::Matrix3d &covariance_matrix)
  {
	  std::vector<Point3D> *points = cloud->getPointCloud();
    // Initialize to 0
    covariance_matrix.setZero ();

    if (indices.size () == 0) 
      return;
    // For each point in the cloud
    for (size_t i = 0; i < indices.size (); ++i)
    {
      // Check if the point is invalid
      if (isnan (points->data()[indices[i]].getX()) || isnan (points->data()[indices[i]].getY()) || isnan (points->data()[indices[i]].getZ()))
        continue;

      Point3D p;
      p.setX(points->data()[indices[i]].getX() - centroid[0]);
      p.setY(points->data()[indices[i]].getY() - centroid[1]);
      p.setZ(points->data()[indices[i]].getZ() - centroid[2]);

      double demean_xy = p.getX() * p.getY();
      double demean_xz = p.getX() * p.getZ();
      double demean_yz = p.getY() * p.getZ();

      covariance_matrix (0, 0) += p.getX() * p.getX();
      covariance_matrix (0, 1) += demean_xy;
      covariance_matrix (0, 2) += demean_xz;
      covariance_matrix (1, 0) += demean_xy;
      covariance_matrix (1, 1) += p.getY() * p.getY();
      covariance_matrix (1, 2) += demean_yz;
      covariance_matrix (2, 0) += demean_xz;
      covariance_matrix (2, 1) += demean_yz;
      covariance_matrix (2, 2) += p.getZ() * p.getZ();
    }
  }

/** \brief Solve the eigenvalues and eigenvectors of a given 3x3 covariance matrix, and estimate the least-squares
  * plane normal and surface curvature.
  * \param covariance_matrix the 3x3 covariance matrix
  * \param point a point lying on the least-squares plane (SSE aligned)
  * \param plane_parameters the resultant plane parameters as: a, b, c, d (ax + by + cz + d = 0)
  * \param curvature the estimated surface curvature as a measure of
  * \f[
  * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
  * \f]
  */
inline void
  solvePlaneParameters (const Eigen::Matrix3d &covariance_matrix, const Eigen::Vector4d &point,
                        Eigen::Vector4d &plane_parameters, double &curvature)
{
  // Avoid getting hung on Eigen's optimizers
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (!std::isfinite (covariance_matrix (i, j)))
      {
        //ROS_WARN ("[pcl::solvePlaneParameteres] Covariance matrix has NaN/Inf values!");
        plane_parameters.setConstant (std::numeric_limits<double>::quiet_NaN ());
        curvature = std::numeric_limits<double>::quiet_NaN ();
        return;
      }

  // Extract the eigenvalues and eigenvectors

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
  EIGEN_ALIGN_128 Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
  EIGEN_ALIGN_128 Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

  // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
  // Note: Remember to take care of the eigen_vectors ordering
  double norm = 1.0 / eigen_vectors.col (0).norm ();

  plane_parameters[0] = eigen_vectors (0, 0) * norm;
  plane_parameters[1] = eigen_vectors (1, 0) * norm;
  plane_parameters[2] = eigen_vectors (2, 0) * norm;

  // Hessian form (D = nc . p_plane (centroid here) + p)
  plane_parameters[3] = -1 * (plane_parameters[0] * point[0] +
                              plane_parameters[1] * point[1] +
                              plane_parameters[2] * point[2]);

  // Compute the curvature surface change
  double eig_sum = eigen_values.sum ();
  if (eig_sum != 0)
    curvature = fabs ( eigen_values[0] / eig_sum );
  else
    curvature = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Solve the eigenvalues and eigenvectors of a given 3x3 covariance matrix, and estimate the least-squares
  * plane normal and surface curvature.
  * \param covariance_matrix the 3x3 covariance matrix
  * \param nx the resultant X component of the plane normal
  * \param ny the resultant Y component of the plane normal
  * \param nz the resultant Z component of the plane normal
  * \param curvature the estimated surface curvature as a measure of
  * \f[
  * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
  * \f]
  */
inline void
  solvePlaneParameters (const Eigen::Matrix3d &covariance_matrix,
                        double &nx, double &ny, double &nz, double &curvature)
{
  // Avoid getting hung on Eigen's optimizers
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      if (!std::isfinite (covariance_matrix (i, j)))
      {
        //ROS_WARN ("[pcl::solvePlaneParameteres] Covariance matrix has NaN/Inf values!");
        nx = ny = nz = curvature = std::numeric_limits<double>::quiet_NaN ();
        return;
      }
  // Extract the eigenvalues and eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
  EIGEN_ALIGN_128 Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
  EIGEN_ALIGN_128 Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

  // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
  // Note: Remember to take care of the eigen_vectors ordering
  double norm = 1.0 / eigen_vectors.col (0).norm ();

  nx = eigen_vectors (0, 0) * norm;
  ny = eigen_vectors (1, 0) * norm;
  nz = eigen_vectors (2, 0) * norm;

  // Compute the curvature surface change
  double eig_sum = eigen_values.sum ();
  if (eig_sum != 0)
    curvature = fabs ( eigen_values[0] / eig_sum );
  else
    curvature = 0;
}




}

#endif  //#ifndef PCL_FEATURE_H_
