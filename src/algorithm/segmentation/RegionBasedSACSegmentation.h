/*
 * RegionBasedSACSegmentation.h
 *
 *  Created on: Apr 17, 2011
 *      Author: reon
 */

#ifndef REGIONBASEDSACSEGMENTATION_H_
#define REGIONBASEDSACSEGMENTATION_H_

#include "IObjectModel.h"
#include "ISACMethods.h"
#include "core/PointCloud3D.h"

//Object Models Supported
#include "ObjectModelPlane.h"

//SAC Methods Supported
#include "SACMethodALMeDS.h"
#include "SACMethodRANSAC_ROS.h"
#include "SACMethodMSAC_ROS.h"
#include "SACMethodLMeDS_ROS.h"
#include "SACMethodMLESAC_ROS.h"

namespace BRICS_3D{
class RegionBasedSACSegmentation {

protected:
	/** \brief The underlying data model used (i.e. what is it that we attempt to search for). */
	IObjectModel *objectModel;

	/** \brief The underlying SAC method used. */
	ISACMethods *sacMethod;

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

	/**Defining the object model types ids supported */

	const static int SACMODEL_PLANE = 0;
	const static int SACMODEL_LINE = 1;
	const static int SACMODEL_CIRCLE2D = 2;
	const static int SACMODEL_CIRCLE3D = 3;
	const static int SACMODEL_SPHERE = 4;
	const static int SACMODEL_CYLINDER = 5;
	const static int SACMODEL_CONE = 6;
	const static int SACMODEL_TORUS = 7;
	const static int SACMODEL_ORIENTED_LINE = 8;
	const static int SACMODEL_ORIENTED_PLANE = 9;
	const static int SACMODEL_PARALLEL_LINES = 10;
	const static int SACMODEL_NORMAL_PLANE = 11;

	/** Defining the SAC methods supported*/
	const static int SAC_ALMeDS = 0;
	const static int SAC_RANSAC = 1;
	const static int SAC_LMEDS = 2;
	const static int SAC_MSAC = 3;
	const static int SAC_MLESAC = 4;

	/**
	 * Default Constructor
	 */
	RegionBasedSACSegmentation(){
		this->threshold = -1;
		this->maxIterations = 10000;
		this->probability = 0.99;
	}
	/**
	 * \brief Set the pointcloud to be processed
	 * \param Pointcloud to be processed
	 */
	inline void setInputPointCloud(PointCloud3D *input) {
		this->inputPointCloud = input;

	}

	/** \brief Set the distance to model threshold.
	 * \param threshold distance to model threshold
	 */
	inline void setDistanceThreshold(double threshold) {
		this->threshold = threshold;
	}

	/** \brief Get the distance to model threshold, as set by the user. */
	inline double getDistanceThreshold() {
		return (this->threshold);
	}

	/** \brief Set the maximum number of iterations.
	 * \param max_iterations maximum number of iterations
	 */
	inline void setMaxIterations(int maxIterations) {
		this->maxIterations = maxIterations;
	}

	/** \brief Get the maximum number of iterations, as set by the user. */
	inline int getMaxIterations() {
		return (maxIterations);
	}

	/** \brief Set the desired probability of choosing at least one sample free from outliers.
	 * \param probability the desired probability of choosing at least one sample free from outliers
	 *\\ToDo
	 *\note internally, the probability is set to 99% (0.99) by default.
	 */
	inline void setProbability(double probability) {
		this->probability = probability;
	}

	/** \brief Obtain the probability of choosing at least one sample free from outliers, as set by the user. */
	inline double getProbability() {
		return (probability);
	}

	/** \brief Return the best model found so far.
	 * \param model the resultant model
	 */
	inline void getModel(std::vector<int> &model) {
		model = this->model;
	}

	/** \brief Return the best set of inliers found so far for this model.
	 * \param inliers the resultant set of inliers
	 */
	inline void getInliers(std::vector<int> &inliers) {
		inliers = this->inliers;
	}

	/** \brief Return the model coefficients of the best model found so far.
	 * \param model_coefficients the resultant model coefficients
	 */
	inline void getModelCoefficients(Eigen::VectorXf &modelCoefficients) {
		modelCoefficients = this->modelCoefficients;
	}

	/** \brief The type of model to use (user given parameter).
	 * \param model the model type (check \a model_types.h)
	 */
	inline void setModelType(int modelType) {
		this->modelType = modelType;
	}

	/** \brief Get the type of SAC model used. */
	inline int getModelType() {
		return (this->modelType);
	}

	/** \brief The type of sample consensus method to use (user given parameter).
	 * \param method the method type (check \a method_types.h)
	 */
	inline void setMethodType(int methodType) {
		this->SACMethodType = methodType;
	}

	/** \brief Get the type of sample consensus method used. */
	inline int getMethodType() {
		return (this->SACMethodType);
	}

	/** \brief Set to true if a coefficient refinement is required.
	 * \param optimize true for enabling model coefficient refinement, false otherwise
	 */
	inline void setOptimizeCoefficients(bool optimize) {
		this->optimizeCoefficients = optimize;
	}

	/** \brief Get the coefficient refinement internal flag. */
	inline bool getOptimizeCoefficients() {
		return (this->optimizeCoefficients);
	}

	/** \brief Initialize the Sample Consensus model and set its parameters.
	 * \param model_type the type of SAC model that is to be used
	 */
	virtual inline bool initSACModel(const int model_type) {

		// Build the model
		switch (model_type) {
		case SACMODEL_PLANE: {
			cout << "[initSACModel] Using a model of type: SACMODEL_PLANE"<<endl;
			objectModel = new ObjectModelPlane();
			objectModel->setInputCloud(inputPointCloud);
			break;
		}
		/*case SACMODEL_LINE:
			 {
			 ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_LINE", getName ().c_str ());
			 model_.reset (new SampleConsensusModelLine<PointT> (this->input_));
			 break;
			 }
			 case SACMODEL_CIRCLE2D:
			 {
			 ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_CIRCLE2D", getName ().c_str ());
			 model_.reset (new SampleConsensusModelCircle2D<PointT> (this->input_));
			 break;
			 }
			 case SACMODEL_SPHERE:
			 {
			 ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_SPHERE", getName ().c_str ());
			 model_.reset (new SampleConsensusModelSphere<PointT> (this->input_));
			 break;
			 }
			 case SACMODEL_ORIENTED_LINE:
			 {
			 ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_ORIENTED_LINE", getName ().c_str ());
			 model_.reset (new SampleConsensusModelOrientedLine<PointT> (this->input_));
			 break;
			 }
			 case SACMODEL_ORIENTED_PLANE:
			 {
			 ROS_DEBUG ("[pcl::%s::initSACModel] Using a model of type: SACMODEL_ORIENTED_PLANE", getName ().c_str ());
			 model_.reset (new SampleConsensusModelOrientedPlane<PointT> (this->input_));
			 break;
			 }*/
		default: {
			cout << "[initSACModel] No valid model given!";
			return (false);
		}
		}
		return (true);
	}

	/** \brief Initialize the Sample Consensus method and set its parameters.
	 * \param method_type the type of SAC method to be used
	 */
	virtual inline void initSACMethod(const int method_type) {
		// Build the sample consensus method
		switch (method_type) {
		case SAC_ALMeDS: {
			cout<<"[initSAC] Using a method of type: SAC_ALMeDS with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodALMeDS();
			sacMethod->setObjectModel(objectModel);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_RANSAC:
		{
			cout<< "[initSAC] Using a method of type: SAC_RANSAC_ROS with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodRANSAC_ROS();
			sacMethod->setObjectModel(objectModel);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_LMEDS: {
			cout<<"[initSAC] Using a method of type: SAC_LMeDS_ROS with a model threshold of "<<threshold<<endl;
						sacMethod = new SACMethodLMeDS_ROS();
						sacMethod->setObjectModel(objectModel);
						sacMethod->setDistanceThreshold(threshold);
						sacMethod->setPointCloud(inputPointCloud);
						break;
		}
		case SAC_MSAC: {
			cout<<"[initSAC] Using a method of type: SAC_MSAC_ROS with a model threshold of "<<threshold<<endl;
						sacMethod = new SACMethodMSAC_ROS();
						sacMethod->setObjectModel(objectModel);
						sacMethod->setDistanceThreshold(threshold);
						sacMethod->setPointCloud(inputPointCloud);
							break;
		}
		case SAC_MLESAC: {
			cout<<"[initSAC] Using a method of type: SAC_MLESAC_ROS with a model threshold of "<<threshold<<endl;
						sacMethod = new SACMethodMLESAC_ROS();
						sacMethod->setObjectModel(objectModel);
						sacMethod->setDistanceThreshold(threshold);
						sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		}
		// Set the Sample Consensus parameters if they are given/changed
		if (sacMethod->getProbability() != probability) {
			cout<<"[initSAC] Setting the desired probability to "<< this->probability<<endl;
			sacMethod->setProbability(probability);
		}

		if (maxIterations!= -1 && sacMethod->getMaxIterations()
				!= maxIterations) {
			cout<<"[initSAC] Setting the maximum number of iterations to "<<maxIterations<<endl;
			sacMethod->setMaxIterations(maxIterations);
		}
	}


	/** \brief Base method for segmentation of a model in a PointCloud given by <setInputCloud (), setIndices ()>
	 * \param inliers the resultant point indices that support the model found (inliers)
	 * \param model_coefficients the resultant model coefficients
	 */
	virtual void
	segment (std::vector<int> &inliers, Eigen::VectorXf &model_coefficients)
	{

		// Initialize the Sample Consensus model and set its parameters
		if (!initSACModel (modelType))
		{
			cout<<"[segment] Error initializing the SAC model!"<<endl;
			return;
		}

		// Initialize the Sample Consensus method and set its parameters
		initSACMethod(SACMethodType);

		//Compute the model

		if (!sacMethod->computeModel())
		{
			cout<<"[segment] Error segmenting the model! No solution found"<<endl;
		}


		// Get the model inliers
		sacMethod->getInliers(inliers);


		// Get the model coefficients
		Eigen::VectorXf coeff;
		sacMethod->getModelCoefficients (coeff);


		// If the user needs optimized coefficients
		if (optimizeCoefficients)
		{
			Eigen::VectorXf coeff_refined;

			objectModel->optimizeModelCoefficients(inliers,coeff,coeff_refined);
			this->modelCoefficients =coeff_refined;
			// Refine inliers
			objectModel->selectWithinDistance(coeff_refined,threshold,inliers);

		}
		else
		{
			this->modelCoefficients = coeff;

		}

		model_coefficients=this->modelCoefficients;

	}
};
}
#endif /* REGIONBASEDSACSEGMENTATION_H_ */
