/*
 * @file: ISACMethods.h
 *
 * @date: Apr 19, 2011
 * @author: reon
 */


#ifndef ISACMETHODS_H_
#define ISACMETHODS_H_

#include "algorithm/segmentation/objectModels/IObjectModel.h"
#include "core/PointCloud3D.h"
#include <algorithm>
#include <float.h>
namespace BRICS_3D {

class ISACMethods {
protected:
	/** @brief The object model to be searched for in its parametric form. */
	IObjectModel *objectModel;

	/** @brief Threshold distance to model. Default value -1  */
	double threshold;

	/** @brief Maximum number of iterations for model estimation. Default value 10000 */
	int maxIterations;

	/** @brief Probability of choosing at least one sample free from outliers. Default value 0.99*/
	double probability;

	/** @brief  The point indices corresponding to the model computed in the last iteration*/
	std::vector<int> inliers;

	/** @brief The estimated model coefficients */
	Eigen::VectorXd modelCoefficients;

	/** @brief No of iterations for model estimation already completed */
	int iterations;

	/** @brief Indicates the object model estimated*/
	int modelType;

	/** @brief Indicates the sample consensus method used for estimating
	 * the model
	 * */
	int SACMethodType;

	/** @brief Indicates if model coefficient refinement is enabled*/
	bool optimizeCoefficients;

	/** @brief The input point-cloud to be processed*/
	PointCloud3D* inputPointCloud;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//	ISACMethods();


	/** @brief Set the object model to be used
	 *  @param pointer to the object model
	 */
	inline void
	setObjectModel (IObjectModel *objectModel)
	{
		this->objectModel = objectModel;
	}



	/** @brief Set the pointcloud to be used
	 *  @param pointer to the input pointcloud
	 */
	inline void
	setPointCloud (PointCloud3D* inputPointCloud)
	{
		this->inputPointCloud = inputPointCloud;
	}

	/** @brief Set the user-defined threshold distance to the model.
	 *  @param threshold user-defined thresholddistance to model threshold
	 */
	inline void
	setDistanceThreshold (double threshold)
	{
		this->threshold = threshold;
	}


	/** @brief Get the distance to model threshold */
	inline double
	getDistanceThreshold ()
	{
		return (this->threshold);
	}


	/** @brief Set the maximum number of iterations for model estimation.
	 * @param max_iterations maximum number of iterations  for model estimation
	 */
	inline void
	setMaxIterations (int maxIterations)
	{
		this->maxIterations = maxIterations;
	}


	/** @brief Get the maximum number of iterations for model estimation */
	inline int
	getMaxIterations ()
	{
		return (maxIterations);
	}


	/** @brief Set the desired probability of choosing at least one sample free from outliers.
	 *  @param probability the desired probability of choosing at least one sample free from outliers
	 */
	inline void
	setProbability (double probability)
	{
		this->probability = probability;
	}


	/** @brief Obtain the probability of choosing at least one sample free from outliers */
	inline double
	getProbability ()
	{
		return (probability);
	}


	/**
	 * @brief Compute the actual model. Pure virtual.
	 * */
	virtual bool
	computeModel () = 0;


	/** @brief Get a set of randomly selected indices.
	 *  @param indices the input indices vector
	 *  @param noSamples the desired number of point indices to randomly select
	 *  @param randomSamples the resultant output set of randomly selected indices
	 */
	inline void
	getRandomSamples (std::set<int> &indices, size_t noSamples, std::set<int> &randomSamples)
	{
		randomSamples.clear ();
		while (randomSamples.size () < noSamples)
			randomSamples.insert ((int) (indices.size() * (rand () / (RAND_MAX + 1.0))));
	}


	/** @brief Return the best set of inliers found so far for this model.
	 *  @param inliers the resultant set of inliers found
	 */
	inline void
	getInliers (std::vector<int> &inliers)
	{
		inliers = this->inliers;
	}


	/** @brief Return the model coefficients of the best model found so far.
	 *  @param modelCoefficients the resultant model coefficients
	 */
	inline void
	getModelCoefficients (Eigen::VectorXd &modelCoefficients)
	{
		modelCoefficients = this->modelCoefficients;
	}


	/** @brief Set the type of the model to be estimated
	 *  @param modelType the model type
	 */
	inline void
	setModelType (int modelType)
	{
		this->modelType = modelType;
	}

	/** @brief Get the type of SAC model to beestimated. */
	inline int
	getModelType ()
	{
		return (this->modelType);
	}


	/** @brief Set the type of sample consensus method to be used
	 * @param methodType the method type
	 */
	inline void
	setMethodType (int methodType)
	{
		this->SACMethodType = methodType;
	}


	/** @brief Get the type of sample consensus method used. */
	inline int
	getMethodType ()
	{
		return (this->SACMethodType);
	}


	/** @brief Set to true if a coefficient refinement is required.
	 *  @param optimize true for enabling model coefficient refinement, false otherwise
	 */
	inline void
	setOptimizeCoefficients (bool optimize)
	{
		this->optimizeCoefficients = optimize;
	}


	/** @brief Get the coefficient refinement internal flag. */
	inline bool
	getOptimizeCoefficients ()
	{
		return (this->optimizeCoefficients);
	}


//	virtual ~ISACMethods();
};

}


#endif /* ISACMETHODS_H_ */
