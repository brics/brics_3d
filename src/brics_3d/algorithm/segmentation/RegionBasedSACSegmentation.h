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

#ifndef BRICS_3D_REGIONBASEDSACSEGMENTATION_H_
#define BRICS_3D_REGIONBASEDSACSEGMENTATION_H_

#include "brics_3d/algorithm/segmentation/ISegmentation.h"
#include "brics_3d/algorithm/segmentation/objectModels/IObjectModel.h"
#include "brics_3d/algorithm/segmentation/objectModels/IObjectModelUsingNormals.h"
#include "brics_3d/algorithm/segmentation/SACMethods/ISACMethods.h"
#include "brics_3d/core/PointCloud3D.h"

//Object Models Supported
#include "brics_3d/algorithm/segmentation/objectModels/ObjectModelPlane.h"
#include "brics_3d/algorithm/segmentation/objectModels/ObjectModelPlaneFromLines.h"
#include "brics_3d/algorithm/segmentation/objectModels/ObjectModelPlaneFromLineAndPoint.h"
#include "brics_3d/algorithm/segmentation/objectModels/ObjectModelOrientedPlane.h"
#include "brics_3d/algorithm/segmentation/objectModels/ObjectModelNormalPlane.h"
#include "brics_3d/algorithm/segmentation/objectModels/ObjectModelCircle.h"
#include "brics_3d/algorithm/segmentation/objectModels/ObjectModelCylinder.h"
#include "brics_3d/algorithm/segmentation/objectModels/ObjectModelOrientedLine.h"
#include "brics_3d/algorithm/segmentation/objectModels/ObjectModelLine.h"
#include "brics_3d/algorithm/segmentation/objectModels/ObjectModelSphere.h"

//SAC Methods Supported
#include "brics_3d/algorithm/segmentation/SACMethods/SACMethodALMeDS.h"
#include "brics_3d/algorithm/segmentation/SACMethods/SACMethodRANSAC.h"
#include "brics_3d/algorithm/segmentation/SACMethods/SACMethodMSAC.h"
#include "brics_3d/algorithm/segmentation/SACMethods/SACMethodLMeDS.h"
#include "brics_3d/algorithm/segmentation/SACMethods/SACMethodMLESAC.h"

namespace brics_3d{

/**
 * @brief Adapter class for initialization of sample consensus based segmentation method.
 * @ingroup segmentation
 */
class RegionBasedSACSegmentation : public ISegmentation {

protected:
	/** @brief The object model to be searched for in its parametric form. */
	IObjectModel *objectModel;

	/** @brief The sample consensus method to be used for model parameter estimation. */
	ISACMethods *sacMethod;

	/** @brief Threshold distance to model. Default value -1  */
	double threshold;

	/** @brief Maximum number of iterations for model estimation. Default value 10000 */
	int maxIterations;

	/** @brief Probability of choosing at least one sample free from outliers. Default value 0.99*/
	double probability;

	/** @brief The model computed in the last iteration */
	std::vector<int> model;

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

//	/** @brief The input point-cloud to be processed*/
//	PointCloud3D* inputPointCloud;

public:

	/**Defining the object model types supported */

	const static int OBJMODEL_PLANE = 0;
	const static int OBJMODEL_LINE = 1;
	const static int OBJMODEL_CIRCLE = 2;
	const static int OBJMODEL_SPHERE = 3;
	const static int OBJMODEL_ORIENTED_LINE = 5;
	const static int OBJMODEL_ORIENTED_PLANE = 6;
	const static int OBJMODEL_PLANE_FROM_LINES = 8;
	const static int OBJMODEL_PLANE_FROM_LINE_AND_POINT = 9;

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

		this->objectModel = 0;
		this->sacMethod = 0;
	}
//	/**
//	 * @brief Set the input pointcloud to be processed
//	 * @param Pointcloud input pointcloudto be processed
//	 */
//	inline void setInputPointCloud(PointCloud3D *input) {
//		this->inputPointCloud = input;
//
//	}

	/** @brief Set the distance to model threshold.
	 * @param threshold distance to model threshold
	 */
	inline void setDistanceThreshold(double threshold) {
		this->threshold = threshold;
	}

	/** @brief Get the distance to model threshold, as set by the user. */
	inline double getDistanceThreshold() {
		return (this->threshold);
	}

	/** @brief Set the Maximum number of iterations for model estimation
	 *  @param max_iterations maximum number of iterations for model estimation
	 */
	inline void setMaxIterations(int maxIterations) {
		this->maxIterations = maxIterations;
	}

	/** @brief Get the maximum number of iterations, as set by the user. */
	inline int getMaxIterations() {
		return (maxIterations);
	}

	/** @brief Set the desired probability of choosing at least one sample free from outliers.
	 * @param probability the desired probability of choosing at least one sample free from outliers
	 *\note internally, the probability is set to 99% (0.99) by default.
	 */
	inline void setProbability(double probability) {
		this->probability = probability;
	}

	/** @brief Obtain the probability of choosing at least one sample free from outliers, as set by the user. */
	inline double getProbability() {
		return (probability);
	}

	/** @brief Return the best model found so far.
	 * @param model the resultant model
	 */
	inline void getModel(std::vector<int> &model) {
		model = this->model;
	}

	/** @brief Return the best set of inliers found so far for this model.
	 * @param inliers the resultant set of inliers
	 */
	inline void getInliers(std::vector<int> &inliers) {
		inliers = this->inliers;
	}

	/** @brief Return the model coefficients of the best model found so far.
	 * @param model_coefficients the resultant model coefficients
	 */
	inline void getModelCoefficients(Eigen::VectorXd &modelCoefficients) {
		modelCoefficients = this->modelCoefficients;
	}

	/** @brief The type of model to use (user given parameter).
	 * @param model the model type (check \a model_types.h)
	 */
	inline void setModelType(int modelType) {
		this->modelType = modelType;
	}

	/** @brief Get the type of SAC model used. */
	inline int getModelType() {
		return (this->modelType);
	}

	/** @brief The type of sample consensus method to use (user given parameter).
	 * @param method the method type (check \a method_types.h)
	 */
	inline void setMethodType(int methodType) {
		this->SACMethodType = methodType;
	}

	/** @brief Get the type of sample consensus method used. */
	inline int getMethodType() {
		return (this->SACMethodType);
	}


	/** @brief Initialize the Sample Consensus model and set its parameters.
	 *  @param model_type the type of SAC model that is to be used
	 */
	inline bool initSACModel(const int modelType) {

		// Build the model
		switch (modelType) {
		case OBJMODEL_PLANE: {
			cout << "[SAC Segmentation] Using a model of type: OBJECT_MODEL_PLANE"<<endl;
			objectModel = new ObjectModelPlane();
			objectModel->setInputCloud(inputPointCloud);
			break;
		}
		case OBJMODEL_CIRCLE:
		{
			cout << "[SAC Segmentation] Using a model of type: OBJECT_MODEL_CIRCLE"<<endl;
			objectModel = new ObjectModelCircle();
			objectModel->setInputCloud(inputPointCloud);
			break;

		}
		case OBJMODEL_SPHERE:
		{
			cout << "[SAC Segmentation] Using a model of type: OBJECT_MODEL_SPHERE"<<endl;
			objectModel = new ObjectModelSphere();
			objectModel->setInputCloud(inputPointCloud);
			break;
		}
		case OBJMODEL_LINE:
		{
			cout << "[SAC Segmentation] Using a model of type: OBJECT_MODEL_LINE"<<endl;
			objectModel = new ObjectModelLine();
			objectModel->setInputCloud(inputPointCloud);
			break;
		}
		case OBJMODEL_ORIENTED_LINE:
		{
			//ToDo check the initialization
			cout << "[initSACModel] Using a model of type: OBJECT_MODEL_SPHERE"<<endl;
			objectModel = new ObjectModelOrientedLine();
			objectModel->setInputCloud(inputPointCloud);
			break;
		}
		case OBJMODEL_ORIENTED_PLANE:
		{
			//ToDo check the initialization
			cout << "[SAC Segmentation] Using a model of type: OBJECT_MODEL_PLANE"<<endl;
			objectModel = new ObjectModelOrientedPlane();
			objectModel->setInputCloud(inputPointCloud);
			break;
		}
		case OBJMODEL_PLANE_FROM_LINES:
		{
			//ToDo check the initialization
			cout << "[SAC Segmentation] Using a model of type: OBJECT_MODEL_PLANE_FROM_LINES"<<endl;
			objectModel = new ObjectModelPlaneFromLines();
			objectModel->setInputCloud(inputPointCloud);
			break;
		}
		case OBJMODEL_PLANE_FROM_LINE_AND_POINT:
		{
			//ToDo check the initialization
			cout << "[SAC Segmentation] Using a model of type: OBJECT_MODEL_PLANE_FROM_LINE_AND_POINT"<<endl;
			objectModel = new ObjectModelPlaneFromLineAndPoint;
			objectModel->setInputCloud(inputPointCloud);
			break;
		}
		default: {
			cout << "[initSACModel] No valid model given!";
			return (false);
		}
		}
		return (true);
	}

	/** @brief Initialize the Sample Consensus method and set its parameters.
	 * @param method_type the type of SAC method to be used
	 */
	inline void initSACMethod(const int method_type) {

		switch (method_type) {
		case SAC_ALMeDS: {
			cout<<"[initSAC] Using a method of type: SAC_ALMeDS with a model threshold of "<<threshold<<endl;
			if (sacMethod == 0){
				sacMethod = new SACMethodALMeDS();
			}
			sacMethod->setObjectModel(objectModel);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_RANSAC:
		{
			cout<< "[SAC Segmentation] Using a method of type: SAC_RANSAC with a model threshold of "<<threshold<<endl;
			if (sacMethod == 0){
				sacMethod = new SACMethodRANSAC();
			}
			sacMethod->setObjectModel(objectModel);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_LMEDS: {
			cout<<"[SAC Segmentation] Using a method of type: SAC_LMeDS with a model threshold of "<<threshold<<endl;
			if (sacMethod == 0){
				sacMethod = new SACMethodLMeDS();
			}
			sacMethod->setObjectModel(objectModel);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_MSAC: {
			cout<<"[SAC Segmentation] Using a method of type: SAC_MSAC with a model threshold of "<<threshold<<endl;
			if (sacMethod == 0){
				sacMethod = new SACMethodMSAC();
			}
			sacMethod->setObjectModel(objectModel);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_MLESAC: {
			cout<<"[SAC Segmentation] Using a method of type: SAC_MLESAC with a model threshold of "<<threshold<<endl;
			if (sacMethod == 0){
				sacMethod = new SACMethodMLESAC();
			}
			sacMethod->setObjectModel(objectModel);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		}
		// Set the Sample Consensus parameters if they are given/changed
		if (sacMethod->getProbability() != probability) {
			cout<<"[SAC Segmentation] Setting the desired probability to "<< this->probability<<endl;
			sacMethod->setProbability(probability);
		}

		if (maxIterations!= -1 && sacMethod->getMaxIterations()
				!= maxIterations) {
			cout<<"[SAC Segmentation] Setting the maximum number of iterations to "<<maxIterations<<endl;
			sacMethod->setMaxIterations(maxIterations);
		}
	}


	/** @brief Base method for finding of a model in the input point cloud
	 *  @param inliers the point indices lying inside the estimated model
	 *  @param model_coefficients the resultant model coefficients
	 */
	int
	segment ()
	{
		assert(this->inputPointCloud != 0);

		// Initialize the Sample Consensus model and set its parameters
		if (!initSACModel (modelType))
		{
			cout<<"[SAC Segmentation] Error initializing the SAC model!"<<endl;
			return 0;
		}

		// Initialize the Sample Consensus method and set its parameters
		//if (sacMethod == 0){
			initSACMethod(SACMethodType);
		//}

		//Compute the model

		if (!sacMethod->computeModel())
		{
			cout<<"[SAC Segmentation] Error segmenting the model! No solution found"<<endl;
		}


		// Get the model inliers
		sacMethod->getInliers(inliers);


		// Get the model coefficients
		sacMethod->getModelCoefficients (this->modelCoefficients);

		return 1;

	}
};
}
#endif /* BRICS_3D_REGIONBASEDSACSEGMENTATION_H_ */
