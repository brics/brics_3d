/*
 * IObjectModel.h
 *
 *  Created on: Apr 17, 2011
 *      Author: reon
 */

#ifndef IOBJECTMODEL_H_
#define IOBJECTMODEL_H_

#include "core/PointCloud3D.h"
#include <Eigen/Geometry>
#include <set>
#include "algorithm/segmentation/features/ROSnormal3D.h"

using namespace std;

namespace BRICS_3D{
/*
 * Abstract Interface for representing 3D shapes.
 */
class IObjectModel {


protected:

	/** \brief Represents the point-cloud to be processed*/
	PointCloud3D* inputPointCloud;

	/** \brief Represents the indices of the point-cloud to be processed. Currently not used*/
	vector<int> *indices;

	/** \brief The minimum and maximum radius limits for the model. Applicable to all models that estimate a radius. */
	double radiusMin, radiusMax;

public:
	IObjectModel(){};


	/** \brief Get a set of random data samples and return them as point indices. Pure virtual.
	 * \param iterations the internal number of iterations used by SAC methods
	 * \param samples the resultant model samples
	 */
	virtual void getSamples (int &iterations, std::vector<int> &samples) = 0;

	/** \brief Check whether the given index samples can form a valid model, compute the model coefficients from
	 * these samples and store them internally in model_coefficients_. Pure virtual.
	 * \param samples the point indices found as possible good candidates for creating a valid model
	 * \param model_coefficients the computed model coefficients
	 */
	virtual bool computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXf &model_coefficients)= 0;


	/** \brief Recompute the model coefficients using the given inlier set and return them to the user. Pure virtual.
	 * @note: these are the coefficients of the model after refinement (e.g., after a least-squares optimization)
	 * \param inliers the data inliers supporting the model
	 * \param model_coefficients the initial guess for the model coefficients
	 * \param optimized_coefficients the resultant recomputed coefficients after non-linear optimization
	 */
	virtual void optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
			Eigen::VectorXf &optimized_coefficients)= 0;


	/** \brief Compute all distances from the cloud data to a given model. Pure virtual.
	 * \param model_coefficients the coefficients of a model that we need to compute distances to
	 * \param distances the resultant estimated distances
	 */
	virtual void getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<double> &distances)= 0;


	/** \brief Select all the points which respect the given model coefficients as inliers. Pure virtual.
	 * \param model_coefficients the coefficients of a model that we need to compute distances to
	 * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
	 * \param inliers the resultant model inliers
	 */
	virtual void selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold,
			std::vector<int> &inliers)= 0;

	/** \brief Select all the points which respect the given model coefficients as inliers. Pure virtual.
	 * \param model_coefficients the coefficients of a model that we need to compute distances to
	 * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
	 * \param distances of inliers from the model
	 */
	virtual void getInlierDistance (std::vector<int> &inliers,
			const Eigen::VectorXf &model_coefficients,  std::vector<double> &distances)= 0;


	/** \brief Create a new point cloud with inliers projected onto the model. Pure virtual.
	 * \param inliers the data inliers that we want to project on the model
	 * \param model_coefficients the coefficients of a model
	 * \param projected_points the resultant projected points
	 * \param copy_data_fields set to true (default) if we want the \a projected_points cloud to be an exact copy
	 *        of the input dataset minus the point projections on the plane model
	 */
	virtual void projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
			PointCloud3D* projectedPointCloud) = 0;

	/** \brief Verify whether a subset of indices verifies a given set of model coefficients. Pure virtual.
	 * \param indices the data indices that need to be tested against the model
	 * \param model_coefficients the set of model coefficients
	 * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
	 */
	virtual bool doSamplesVerifyModel (const std::set<int> &indices,
			const Eigen::VectorXf &model_coefficients, double threshold) = 0;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Provide a pointer to the input dataset
	 * \param cloud the const boost shared pointer to a PointCloud message
	 */
	inline void
	setInputCloud (PointCloud3D* cloud)
	{
		this->inputPointCloud = cloud;

	}

	/**
	 *\brief Set the input cloud to be used and the indices to be used
	 * \param input point cloud
	 * \param indices of the pointcloud to be used
	 */
	inline void
		setInputCloud (PointCloud3D* cloud,vector<int> *indices )
		{
			this->inputPointCloud = cloud;
			this->indices= indices;

		}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Get a pointer to the input point cloud dataset. */
	inline PointCloud3D* getInputCloud () { return (inputPointCloud); }

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Provide a pointer to the vector of indices that represents the input data.
	 * \param indices a pointer to the vector of indices that represents the input data.
	 */
	//toDo resolve indicesptr
	//inline void setIndices (const IndicesPtr &indices) { indices_ = indices; }

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Provide the vector of indices that represents the input data.
	 * \param indices the vector of indices that represents the input data.
	 */
	//toDo resolve indicesptr
	//inline void setIndices (std::vector<int> &indices) { indices_ = boost::make_shared <std::vector<int> > (indices); }

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Get a pointer to the vector of indices used. */
	//
	//inline IndicesPtr getIndices () { return (indices_); }

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief Return an unique id for each type of model employed. */
	//virtual int getModelType () = 0;

	/** \brief Set the minimum and maximum allowable radius limits for the model (applicable to models that estimate
	 * a radius)
	 * \param min_radius the minimum radius model
	 * \param max_radius the maximum radius model
	 * \todo change this to set limits on the entire model
	 */
	inline void
	setRadiusLimits (const double &minRadius, const double &maxRadius)
	{
		radiusMin = minRadius;
		radiusMax = maxRadius;
	}


	/** \brief Get the minimum and maximum allowable radius limits for the model as set by the user.
	 * \param min_radius the resultant minimum radius model
	 * \param max_radius the resultant maximum radius model
	 */
	inline void
	getRadiusLimits (double &minRadius, double &maxRadius)
	{
		minRadius = radiusMin;
		maxRadius = radiusMax;
	}

//	virtual ~IObjectModel(){};
};
}
#endif /* IOBJECTMODEL_H_ */
