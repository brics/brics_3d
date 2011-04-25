/*
 * NormalEstimation.h
 *
 *  Created on: Apr 24, 2011
 *      Author: reon
 */

#ifndef NORMALESTIMATION_H_
#define NORMALESTIMATION_H_

#include "core/PointCloud3D.h"
#include "algorithm/segmentation/features/ROSnormal3D.h"

namespace BRICS_3D {

class NormalEstimation {

private:

	/** \brief Values describing the viewpoint ("pinhole" camera model assumed). For per point viewpoints, inherit
	 * from NormalEstimation and provide your own computeFeature (). By default, the viewpoint is set to 0,0,0. */
	float vpx, vpy, vpz;

	/** \brief Placeholder for the 3x3 covariance matrix at each surface patch. */
	EIGEN_ALIGN_128 Eigen::Matrix3f covarianceMatrix;

	/** \brief 16-bytes aligned placeholder for the XYZ centroid of a surface patch. */
	Eigen::Vector4f xyzCentroid;

public:
	NormalEstimation();
	virtual ~NormalEstimation();

	/** \brief Compute the 3D (X-Y-Z) centroid of a set of points using their indices and return it as a 3D vector.
	 * \param cloud the input point cloud
	 * \param indices the point cloud indices that need to be used
	 * \param centroid the output centroid
	 */
	inline void
	compute4DCentroid (PointCloud3D *cloud,Eigen::Vector4f &centroid)
	{
		// Initialize to 0
		centroid.setZero ();

		// For each point in the cloud
		int cp = 0;
		for (size_t i = 0; i < cloud->getSize(); ++i)
		{
			// Check if the point is invalid
			if (isnan (cloud->getPointCloud()->data()[i].getX()) || isnan (cloud->getPointCloud()->data()[i].getY())
					|| isnan (cloud->getPointCloud()->data()[i].getZ()))
				continue;

			float x = cloud->getPointCloud()->data()[i].getX();
			centroid += Eigen::Vector4f::MapAligned (&x);
			cp++;
		}
		centroid /= cp;
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
	computePointNormal (PointCloud3D *cloud, Eigen::Vector4f &plane_parameters, float &curvature)
	{
		if (cloud->getSize() == 0)
		{
			plane_parameters.setConstant (std::numeric_limits<float>::quiet_NaN ());
			curvature = std::numeric_limits<float>::quiet_NaN ();
			return;
		}
		// Estimate the XYZ centroid
		compute3DCentroid (cloud, xyzCentroid);

		// Compute the 3x3 covariance matrix
		computeCovarianceMatrix (cloud, xyzCentroid, covarianceMatrix);

		// Get the plane normal and surface curvature
		solvePlaneParameters (covarianceMatrix, xyzCentroid, plane_parameters, curvature);
	}


	/** \brief Compute the Least-Squares plane fit for a given set of points, and return the estimated plane
	 * parameters together with the surface curvature.
	 * \param cloud the input point cloud
	 * \param nx the resultant X component of the plane normal
	 * \param ny the resultant Y component of the plane normal
	 * \param nz the resultant Z component of the plane normal
	 * \param curvature the estimated surface curvature as a measure of
	 * \f[
	 * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
	 * \f]
	 */
	inline void
	computePointNormal (PointCloud3D *cloud,
			float &nx, float &ny, float &nz, float &curvature)
	{
		if (cloud->getSize() == 0)
		{
			nx = ny = nz = curvature = std::numeric_limits<float>::quiet_NaN ();
			return;
		}
		// Estimate the XYZ centroid
		compute3DCentroid (cloud, xyzCentroid);

		// Compute the 3x3 covariance matrix
		computeCovarianceMatrix (cloud, xyzCentroid, covarianceMatrix);

		// Get the plane normal and surface curvature
		solvePlaneParameters (covarianceMatrix, nx, ny, nz, curvature);
	}


	/** \brief Set the viewpoint.
	 * \param vpx the X coordinate of the viewpoint
	 * \param vpy the Y coordinate of the viewpoint
	 * \param vpz the Z coordinate of the viewpoint
	 */
	inline void
	setViewPoint (float vpx, float vpy, float vpz)
	{
		this->vpx = vpx;
		this->vpy = vpy;
		this->vpz = vpz;
	}


	/** \brief Get the viewpoint. */
	inline void
	getViewPoint (float &vpx, float &vpy, float &vpz)
	{
		vpx = this->vpx;
		vpy = this->vpy;
		vpz = this->vpz;
	}


private:
	/** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
	 * setSearchSurface () and the spatial locator in setSearchMethod ()
	 * \note In situations where not enough neighbors are found, the normal and curvature values are set to -1.
	 * \param output the resultant point cloud model dataset that contains surface normals and curvatures
	 */
/*	void
	computeFeature (PointCloudOut &output)
	{
		// Allocate enough space to hold the results
		// \note This resize is irrelevant for a radiusSearch ().
		std::vector<int> nn_indices (this->k_);
		std::vector<float> nn_dists (this->k_);

		// Iterating over the entire index vector
		for (size_t idx = 0; idx < this->indices_->size (); ++idx)
		{
			if (!searchForNeighbors ((*this->indices_)[idx], this->search_parameter_, nn_indices, nn_dists))
			{
				output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();
				continue;
			}

			computePointNormal (*this->surface_, nn_indices,
					output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2], output.points[idx].curvature);

			flipNormalTowardsViewpoint (this->surface_->points[idx], vpx_, vpy_, vpz_,
					output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);

		}
	}
*/
};

}

#endif /* NORMALESTIMATION_H_ */
