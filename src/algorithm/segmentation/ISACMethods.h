/*
 * ISACMethods.h
 *
 *  Created on: Apr 19, 2011
 *      Author: reon
 */

#ifndef ISACMETHODS_H_
#define ISACMETHODS_H_

#include "IObjectModel.h"
#include "core/PointCloud3D.h"
#include "ObjectModelPlane.h"

namespace BRICS_3D {

class ISACMethods {
protected:
	/** \brief The underlying data model used (i.e. what is it that we attempt to search for). */
	IObjectModel *objectModel;

	/** \brief Distance to model threshold. */
	double threshold;

	/** \brief Maximum number of iterations before giving up. */
	int maxIterations;

	/** \brief Desired probability of choosing at least one sample free from outliers. */
	double probability;

	/** \brief The model found after the last computeModel () as point cloud indices. */
	std::vector<int> model;

	/** \brief The indices of the points that were chosen as inliers after the last computeModel () call. */
	std::vector<int> inliers;

	/** \brief The coefficients of our model computed directly from the model found. */
	Eigen::VectorXf modelCoefficients;

	/** \brief Total number of internal loop iterations that we've done so far. */
	int iterations;

	/** \brief The object model used*/
	int modelType;

	/** \brief SAC Method used*/
	int SACMethodType;

	/** \brief Indicates if model coefficient refinement is enabled*/
	bool optimizeCoefficients;

	/** \brief Represents the point-cloud to be processed*/
	PointCloud3D* inputPointCloud;

public:

//	ISACMethods();


	/** \brief Set the object model to be used
	 * \param pointer to the object model
	 */
	inline void
	setObjectModel (IObjectModel *objectModel)
	{
		this->objectModel = objectModel;
	}



	/** \brief Set the pointcloud to be used
	 * \param pointer to the pointcloud
	 */
	inline void
	setPointCloud (PointCloud3D* inputPointCloud)
	{
		this->inputPointCloud = inputPointCloud;
	}

	/** \brief Set the distance to model threshold.
	 * \param threshold distance to model threshold
	 */
	inline void
	setDistanceThreshold (double threshold)
	{
		this->threshold = threshold;
	}


	/** \brief Get the distance to model threshold, as set by the user. */
	inline double
	getDistanceThreshold ()
	{
		return (this->threshold);
	}


	/** \brief Set the maximum number of iterations.
	 * \param max_iterations maximum number of iterations
	 */
	inline void
	setMaxIterations (int maxIterations)
	{
		this->maxIterations = maxIterations;
	}


	/** \brief Get the maximum number of iterations, as set by the user. */
	inline int
	getMaxIterations ()
	{
		return (maxIterations);
	}


	/** \brief Set the desired probability of choosing at least one sample free from outliers.
	 * \param probability the desired probability of choosing at least one sample free from outliers
	 *\\ToDo
	 *\note internally, the probability is set to 99% (0.99) by default.
	 */
	inline void
	setProbability (double probability)
	{
		this->probability = probability;
	}


	/** \brief Obtain the probability of choosing at least one sample free from outliers, as set by the user. */
	inline double
	getProbability ()
	{
		return (probability);
	}


	/*//Todo Delete the debugVerbosityLevel not included
	 *  \brief Compute the actual model. Pure virtual. */
	virtual bool
	computeModel () = 0;


	/** \brief Get a set of randomly selected indices.
	 * \param indices the input indices vector
	 * \param nr_samples the desired number of point indices to randomly select
	 * \param indices_subset the resultant output set of randomly selected indices
	 */
	inline void
	getRandomSamples (std::set<int> &indices, size_t nr_samples, std::set<int> &indices_subset)
	{
		indices_subset.clear ();
		while (indices_subset.size () < nr_samples)
			indices_subset.insert ((int) (indices.size() * (rand () / (RAND_MAX + 1.0))));
	}


	/** \brief Return the best model found so far.
	 * \param model the resultant model
	 */
	inline void
	getModel (std::vector<int> &model)
	{
		model = this->model;
	}


	/** \brief Return the best set of inliers found so far for this model.
	 * \param inliers the resultant set of inliers
	 */
	inline void
	getInliers (std::vector<int> &inliers)
	{
		inliers = this->inliers;
	}


	/** \brief Return the model coefficients of the best model found so far.
	 * \param model_coefficients the resultant model coefficients
	 */
	inline void
	getModelCoefficients (Eigen::VectorXf &modelCoefficients)
	{
		modelCoefficients = this->modelCoefficients;
	}


	/** \brief The type of model to use (user given parameter).
	 * \param model the model type (check \a model_types.h)
	 */
	inline void
	setModelType (int modelType)
	{
		this->modelType = modelType;
	}

	/** \brief Get the type of SAC model used. */
	inline int
	getModelType ()
	{
		return (this->modelType);
	}


	/** \brief The type of sample consensus method to use (user given parameter).
	 * \param method the method type (check \a method_types.h)
	 */
	inline void
	setMethodType (int methodType)
	{
		this->SACMethodType = methodType;
	}


	/** \brief Get the type of sample consensus method used. */
	inline int
	getMethodType ()
	{
		return (this->SACMethodType);
	}


	/** \brief Set to true if a coefficient refinement is required.
	 * \param optimize true for enabling model coefficient refinement, false otherwise
	 */
	inline void
	setOptimizeCoefficients (bool optimize)
	{
		this->optimizeCoefficients = optimize;
	}


	/** \brief Get the coefficient refinement internal flag. */
	inline bool
	getOptimizeCoefficients ()
	{
		return (this->optimizeCoefficients);
	}


//	virtual ~ISACMethods();
};

}


#endif /* ISACMETHODS_H_ */
