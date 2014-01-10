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


#ifndef BRICS_3D_NORMALESTIMATION_H_
#define BRICS_3D_NORMALESTIMATION_H_

#include <Eigen/Dense>

#include "brics_3d/core/HomogeneousMatrix44.h" // for eigen declarations
#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/NormalSet3D.h"
#include "brics_3d/algorithm/nearestNeighbor/INearestPoint3DNeighbor.h"
#include "brics_3d/algorithm/featureExtraction/INormalEstimation.h"
#include "brics_3d/algorithm/featureExtraction/Centroid3D.h"
#include "brics_3d/algorithm/featureExtraction/Covariance3D.h"

namespace brics_3d {

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

        plane_parameters.setConstant (std::numeric_limits<double>::quiet_NaN ());
        curvature = std::numeric_limits<double>::quiet_NaN ();
        return;
      }

  // Extract the eigenvalues and eigenvectors

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
  EIGEN_ALIGN_MEMORY Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
  EIGEN_ALIGN_MEMORY Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

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
        nx = ny = nz = curvature = std::numeric_limits<double>::quiet_NaN ();
        return;
      }
  // Extract the eigenvalues and eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
  EIGEN_ALIGN_MEMORY Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
  EIGEN_ALIGN_MEMORY Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

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
  EIGEN_ALIGN_MEMORY Eigen::Vector4d xyz_centroid;

  // Estimate the XYZ centroid
  xyz_centroid = Centroid3D::computeCentroid(cloud, *indices);

  // Compute the 3x3 covariance matrix
  covariance_matrix = Covariance3D::computeCovarianceMatrix (cloud, *indices, xyz_centroid);

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
   xyz_centroid = Centroid3D::computeCentroid(cloud, indices);

  // Compute the 3x3 covariance matrix
   covariance_matrix = Covariance3D::computeCovarianceMatrix (cloud, indices, xyz_centroid);

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
	EIGEN_ALIGN_MEMORY Eigen::Vector4d xyz_centroid;


  // Estimate the XYZ centroid
  xyz_centroid = Centroid3D::computeCentroid(cloud, indices);

  // Compute the 3x3 covariance matrix
  covariance_matrix = Covariance3D::computeCovarianceMatrix(cloud, indices, xyz_centroid);

  // Get the plane normal and surface curvature
  solvePlaneParameters (covariance_matrix, nx, ny, nz, curvature);
}


/**
 * @brief Computes the point normals for a given a point cloud.
 * @ingroup featureExtraction
 */
class NormalEstimation : public INormalEstimation {

private:

	/** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
	 * from NormalEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0. */
	double vpx, vpy, vpz;

	/** \brief Number of k-nearest neighbours to be used */
	int k_neighbours;

	/** \brief Represents the point-cloud to be processed*/
	PointCloud3D* inputPointCloud;

	/** \brief Nearest neighbour searching method */
	INearestPoint3DNeighbor *nnSearchMethod;
public:
	NormalEstimation(){
		this->vpx = 0;
		this->vpy = 0;
		this->vpz = 0;
		this->k_neighbours = 10;

	}

	~NormalEstimation(){};

	void estimateNormals(PointCloud3D* pointCloud, NormalSet3D* estimatedNormals) {
		this->setInputCloud(pointCloud);
		this->computeFeature(estimatedNormals);
	}

	/** \brief Get the nearst neighborhood search method. */
	inline INearestPoint3DNeighbor*
	getNNSearchMethod ()
	{
		return (this->nnSearchMethod);
	}


	/** \brief Set the nearst neighborhood search method. */
	inline void
	setSearchMethod (INearestPoint3DNeighbor *nnSearchMethod)
	{	assert(this->inputPointCloud!=NULL);
		this->nnSearchMethod = nnSearchMethod;
		this->nnSearchMethod->setData(this->inputPointCloud);
	}


	/** \brief Provide a pointer to the input dataset
	 * \param cloud the const boost shared pointer to a PointCloud message
	 */
	inline void
	setInputCloud (PointCloud3D* cloud)
	{
		this->inputPointCloud = cloud;
	}

	/** \brief Get a pointer to the input point cloud dataset. */
	inline PointCloud3D* getInputCloud () { return (inputPointCloud); }


	/** \brief Set the number of nearest neighbours to be used
	 * \param k_neighbours: The number of neighbours to be used
	 */
	inline void setkneighbours(int kNeighbours){
		this->k_neighbours = kNeighbours;
	}


	/** \brief Get the number of nearest neighbours to be used
	 */
	inline int getkneighbours(){
		return this->k_neighbours;
	}

	/** \brief Compute the 3D (X-Y-Z) centroid of a set of points using their indices and return it as a 3D vector.
	 * \param cloud the input point cloud
	 * \param indices the point cloud indices that need to be used
	 * \param centroid the output centroid
	 */
	inline void
	compute4DCentroid (PointCloud3D *cloud,Eigen::Vector4d &centroid)
	{
		// Initialize to 0
		centroid.setZero ();

		// For each point in the cloud
		int cp = 0;
		for (size_t i = 0; i < cloud->getSize(); ++i)
		{
			// Check if the point is invalid
			if (isnan ((*cloud->getPointCloud())[i].getX()) || isnan ((*cloud->getPointCloud())[i].getY())
					|| isnan ((*cloud->getPointCloud())[i].getZ()))
				continue;

			double x = (*cloud->getPointCloud())[i].getX();
			centroid += Eigen::Vector4d::MapAligned (&x);
			cp++;
		}
		centroid /= cp;
	}


	/** \brief Set the viewpoint.
	 * \param vpx the X coordinate of the viewpoint
	 * \param vpy the Y coordinate of the viewpoint
	 * \param vpz the Z coordinate of the viewpoint
	 */
	inline void
	setViewPoint (double vpx, double vpy, double vpz)
	{
		this->vpx = vpx;
		this->vpy = vpy;
		this->vpz = vpz;
	}


	/** \brief Get the viewpoint. */
	inline void
	getViewPoint (double &vpx, double &vpy, double &vpz)
	{
		vpx = this->vpx;
		vpy = this->vpy;
		vpz = this->vpz;
	}



	/** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
	 * setSearchSurface () and the spatial locator in setSearchMethod ()
	 * \note In situations where not enough neighbors are found, the normal and curvature values are set to -1.
	 * \param output the resultant point cloud model dataset that contains surface normals and curvatures
	 */
	void
	computeFeature (NormalSet3D *normalSet)
	{
		// Allocate enough space to hold the results
		// \note This resize is irrelevant for a radiusSearch ().
		//using the default number of nearrest-neighbours used.
		//ToDo enable configuration of 'k'
		std::vector<int> nn_indices;
//		nn_indices.resize(k_neighbours);
		double curvature;
		// Iterating over the entire index vector
		nnSearchMethod->setData(this->inputPointCloud);

//		std::vector<Point3D>* points;
//		points = inputPointCloud->getPointCloud();

		for (size_t idx = 0; idx < this->inputPointCloud->getSize(); ++idx)
		{

			nnSearchMethod->findNearestNeighbors(&(*inputPointCloud->getPointCloud())[idx], &nn_indices, k_neighbours);

			if (nn_indices.size()==0)
			{
				normalSet->getNormals()->data()[idx].setX(std::numeric_limits<double>::quiet_NaN ());
				normalSet->getNormals()->data()[idx].setY(std::numeric_limits<double>::quiet_NaN ());
				normalSet->getNormals()->data()[idx].setZ(std::numeric_limits<double>::quiet_NaN ());
				curvature = 0;
				continue;
			}

			double nx,ny,nz;
			computePointNormal (this->inputPointCloud, nn_indices,
					nx,ny,nz,curvature);

			Normal3D tempNormal;
			tempNormal.setX(nx);
			tempNormal.setY(ny);
			tempNormal.setZ(nz);

			normalSet->addNormal(tempNormal);
			flipNormalTowardsViewpoint ((*this->inputPointCloud->getPointCloud())[idx], vpx, vpy, vpz,
					normalSet->getNormals()->data()[idx]);

		}
	}

	  /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
	    * \param point a given point
	    * \param vp_x the X coordinate of the viewpoint
	    * \param vp_y the X coordinate of the viewpoint
	    * \param vp_z the X coordinate of the viewpoint
	    * \param normal the plane normal to be flipped
	    */
	inline void
	    flipNormalTowardsViewpoint (Point3D &point, double vp_x, double vp_y, double vp_z, Normal3D &normal)
	  {
	    // See if we need to flip any plane normals
	    vp_x -= point.getX();
	    vp_y -= point.getY();
	    vp_z -= point.getZ();

	    // Dot product between the (viewpoint - point) and the plane normal
	    double cos_theta = (vp_x * normal.getX() + vp_y * normal.getY() + vp_z * normal.getZ());

	    // Flip the plane normal
	    if (cos_theta < 0)
	    {
	      normal.setX(-1*normal.getX());
	      normal.setY(-1*normal.getY());
	      normal.setZ(-1*normal.getZ());
	      // Hessian form (D = nc . p_plane (centroid here) + p)
	      normal.setZ(-1 * (
	    		  normal.getX() * point.getX() +
	    		  normal.getY() * point.getY() +
	    		  normal.getZ() * point.getZ()));
	    }
	  }

};



}

#endif /* BRICS_3D_NORMALESTIMATION_H_ */
