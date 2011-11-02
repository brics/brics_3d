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

#ifndef IOBJECTMODEL_H_
#define IOBJECTMODEL_H_


#include <Eigen/Geometry>
#include <set>
#include "algorithm/segmentation/features/ROSnormal3D.h"
#include "core/Point3D.h"
#include "core/PointCloud3D.h"
#include <float.h>


using namespace std;

namespace BRICS_3D{
/*
 * @brief Base class for representing 3D shapes. The base functions and interfaces are
 * derived from various open-source libraries which provide segmentation functionalities
 * for example ROS:PCL, MRPT, ITk, etc
 */
class IObjectModel {


protected:

	/** @brief Represents the point-cloud to be processed*/
	PointCloud3D* inputPointCloud;

	/** @brief The minimum and maximum radius limits for the model. Applicable to
	 * all models that estimate a radius. */
	double radiusMin, radiusMax;

	/** @brief Define the maximum number of iterations for collinearity checks */
	const static int MAX_ITERATIONS_COLLINEAR = 1000;

	/** @brief Pointer to the vector of points in the pointcloud**/
	std::vector<Point3D> *points;


public:
	IObjectModel(){};


	/**
	 * @brief returns the minimum number of samples required for a object model generation
	 */
	virtual int getNumberOfSamplesRequired()=0;


	/**
	 * @brief Compute the model coefficients by generating some random samples
	 * @param model_coefficients: the computed model coefficients
	 * @param isDegenerate: Whether degenerate samples are found from
	 * the input pointcloud or not
	 * @param modelFound:	Whether a valid model is found from the sample or not
	 */
	virtual void computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate, bool &modelFound)= 0;


	/** @brief Compute all distances from the cloud data to a given model. Pure virtual.
	 * @param model_coefficients the coefficients of a model
	 * that we need to compute distances to
	 * @param distances the resultant estimated distances
	 */
	virtual void getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances)= 0;


	/** @brief Select all the points which respect the given model coefficients as inliers. Pure virtual.
	 * @param model_coefficients the coefficients of a model that we need to compute distances to
	 * @param threshold a maximum admissible distance threshold for determining the inliers from the outliers
	 * @param inliers the resultant model inliers
	 */
	virtual void selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold,
			std::vector<int> &inliers)= 0;

	/** @brief Select all the points which respect the given model coefficients as inliers. Pure virtual.
	 * @param model_coefficients the coefficients of a model that we need to compute distances to
	 * @param threshold a maximum admissible distance threshold for determining the inliers from the outliers
	 * @param distances of inliers from the model
	 */
	virtual void getInlierDistance (std::vector<int> &inliers,
			const Eigen::VectorXd &model_coefficients,  std::vector<double> &distances)= 0;


	/** @brief Verify whether a subset of indices verifies a given set of model coefficients. Pure virtual.
	 * @param indices the data indices that need to be tested against the model
	 * @param model_coefficients the set of model coefficients
	 * @param threshold a maximum admissible distance threshold for determining the inliers from the outliers
	 */
	virtual bool doSamplesVerifyModel (const std::set<int> &indices,
			const Eigen::VectorXd &model_coefficients, double threshold) = 0;

	/** @brief Provide a pointer to the input dataset
	 * @param cloud pointer to BRICS::PointCloud3D
	 */
	inline void
	setInputCloud (PointCloud3D* cloud)
	{
		this->inputPointCloud = cloud;
		this->points = inputPointCloud->getPointCloud();
	}

	/** @brief Get a pointer to the input point cloud dataset. */
	inline PointCloud3D* getInputCloud () { return (inputPointCloud); }


	/** @brief Set the minimum and maximum allowable radius limits for the model (applicable to models that estimate
	 * a radius)
	 * @param min_radius the minimum radius model
	 * @param max_radius the maximum radius model
	 * todo change this to set limits on the entire model
	 */
	inline void
	setRadiusLimits (const double &minRadius, const double &maxRadius)
	{
		radiusMin = minRadius;
		radiusMax = maxRadius;
	}


	/** @brief Get the minimum and maximum allowable radius limits for the model as set by the user.
	 * @param min_radius the resultant minimum radius model
	 * @param max_radius the resultant maximum radius model
	 */
	inline void
	getRadiusLimits (double &minRadius, double &maxRadius)
	{
		minRadius = radiusMin;
		maxRadius = radiusMax;
	}


	/** @brief Compute the smallest angle between two vectors in the [ 0, PI ) interval in 3D.
	    * @param v1 the first 3D vector (represented as a \a Eigen::Vector4d)
	    * @param v2 the second 3D vector (represented as a \a Eigen::Vector4d)
	    */
	  inline double
	    getAngle3D (const Eigen::Vector4d &v1, const Eigen::Vector4d &v2)
	  {
	    // Compute the actual angle
	    double rad = v1.dot (v2) / sqrt (v1.squaredNorm () * v2.squaredNorm ());
	    if (rad < -1.0) rad = -1.0;
	    if (rad >  1.0) rad = 1.0;
	    return acos (rad);

	    return (rad);
	  }
};
}
#endif /* IOBJECTMODEL_H_ */
