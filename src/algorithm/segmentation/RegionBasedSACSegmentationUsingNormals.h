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

#ifndef REGIONBASEDSACSEGMENTATIONUSINGNORMALS_H_
#define REGIONBASEDSACSEGMENTATIONUSINGNORMALS_H_

#include <vector>
#include <Eigen/Geometry>
#include "core/NormalSet3D.h"
#include "RegionBasedSACSegmentation.h"
#include "algorithm/segmentation/objectModels/ObjectModelCylinder.h"
#include "algorithm/segmentation/objectModels/ObjectModelNormalPlane.h"


namespace brics_3d {

/**
 * @brief: Adapter class for initialization of sample consensus based segmentation method
 * which requires point normals
 */
class RegionBasedSACSegmentationUsingNormals : public RegionBasedSACSegmentation{

private:

	/** @brief The point normal based object model to be searched for in its parametric form. */
	IObjectModelUsingNormals *objectModelUsingNormals;

	/** @brief  Minimum and maximum radius allowable for the models*/
	double radiusMin, radiusMax;

	/** @brief The relative weight (between 0 and 1) to  corresponding to
	 * angular distance between point normals and the plane normal. 0 corresponds to 0 rad and
	 * 1 corresponds to pi/2 rad */
	double normalDistanceWeight;

	/** @brief A pointer to the vector of normals */
	NormalSet3D *normalSet;

	/** @brief Axis along which we need to search for a model perpendicular to */
	Eigen::Vector3d axis;

	/** @brief The maximum angle between the model normal and the given axis */
	double epsAngle;

public:

	const static int OBJMODEL_CYLINDER = 4;
	const static int OBJMODEL_NORMAL_PLANE = 7;

	RegionBasedSACSegmentationUsingNormals(){
		this->threshold = -1;
		this->maxIterations = 10000;
		this->probability = 0.99;
		this->radiusMax=DBL_MAX;
		this->radiusMin=-DBL_MAX;
		this->normalDistanceWeight = 0.1;
		this->epsAngle=0.0;
	};

	~RegionBasedSACSegmentationUsingNormals(){};


	/** @brief Set the pointer to the normal-set for the corresponding input point cloud
	 *  @param normalSet Pointer to brics_3d::NormalSet3D
	 */
	inline void
	setInputNormals (NormalSet3D *normalSet)
	{
		this->normalSet = normalSet;
	}


	/** @brief Get a pointer to brics_3d::NOrmalSet3D for input point cloud being segmented
	 *  @return Pointer to brics_3d::NormalSet3D
	 * */
	inline NormalSet3D*
	getInputNormals ()
	{
		return (this->normalSet);
	}


	/** @brief Set the relative weight variable indicating the angular distance
	 * between point normals and the plane normal.
	 *  @param distanceWeight the distance/angular weight between 0 and 1
	 */
	inline void
	setNormalDistanceWeight (double distanceWeight)
	{
		this->normalDistanceWeight = distanceWeight;
	}


	/** @brief Get the relative weight variable indicating the angular distance
	 * between point normals and the plane normal. */
	inline double
	getNormalDistanceWeight ()
	{
		return (this->normalDistanceWeight);
	}


	/** @brief Set the axis along which we need to search for a model perpendicular to.
	 *  @param axis the axis perpendicular to which the model parameters will be searched for
	 */
	inline void
	setAxis (const Eigen::Vector3d &axis)
	{
		this->axis = axis;
	}


	/** @brief Get the axis along which we need to search for a model perpendicular to. */
	inline Eigen::Vector3d
	getAxis ()
	{
		return (this->axis);
	}


	/** @brief Set the angle epsilon (delta) threshold.
	 *  @param epAngle the maximum allowed difference between the model normal and the given axis.
	 */
	inline void
	setEpsAngle (double epAngle)
	{
		this->epsAngle = epAngle;
	}


	/** @brief Get the angle threshold between the given axis and the model-normal. */
	inline double
	getEpsAngle ()
	{
		return (this->epsAngle);
	}



	/** @brief Set the minimum and maximum radius allowable for models including a radius value
	 *  @param minRadius the minimum radius model
	 *  @param maxRadius the maximum radius model
	 */
	inline void
	setRadiusLimits (const double &minRadius, const double &maxRadius)
	{
		radiusMin = minRadius;
		radiusMax = maxRadius;
	}



	/** @brief Initialize the Sample Consensus model and set its parameters.
	 *  @param model_type the type of SAC model that is to be used
	 */
	inline bool
	initSACModel (const int model_type)
	{
		//Check if input is synced with the normals
		if (this->inputPointCloud->getSize() != this->normalSet->getSize())
		{
			cout<<"[SAC Segmentation] The number of points inthe input point cloud differs than the number of points in the normals!";
			return (false);
		}

		// Build the model
		switch (model_type)
		{
		case OBJMODEL_CYLINDER:
		{
			cout<<"[SAC Segmentation] Using a model of type: OBJMODEL_CYLINDER"<<endl;

			this->objectModelUsingNormals =  new ObjectModelCylinder();
			this->objectModelUsingNormals->setInputCloud(this->inputPointCloud);
			this->objectModelUsingNormals->setInputNormals(this->normalSet);

			// Set the input normals
			double min_radius, max_radius;
			this->objectModelUsingNormals->getRadiusLimits(min_radius,max_radius);
			if (this->radiusMin != min_radius && this->radiusMax!= max_radius)
			{
				cout<<"[SAC Segmentation] Setting radius limits to "<< this->radiusMin<< ", "<<this->radiusMax<<endl;
				this->objectModelUsingNormals->setRadiusLimits (this->radiusMin, this->radiusMax);
			}
			if (this->normalDistanceWeight != this->objectModelUsingNormals->getNormalDistanceWeight ())
			{
				cout<< "[SAC Segmentation] Setting normal distance weight to "<< this->normalDistanceWeight <<endl;
				this->objectModelUsingNormals->setNormalDistanceWeight (this->normalDistanceWeight);
			}
			break;
		}
		case OBJMODEL_NORMAL_PLANE:
		{
			cout<<"[SAC Segmentation] Using a model of type: OBJMODEL_NORMAL_PLANE"<<endl;
			this->objectModelUsingNormals = new ObjectModelNormalPlane();
			this->objectModelUsingNormals->setInputCloud(this->inputPointCloud);
			// Set the input normals
			this->objectModelUsingNormals->setInputNormals(this->normalSet);

			if (normalDistanceWeight != this->objectModelUsingNormals->getNormalDistanceWeight ())
			{
				cout<< "[SAC Segmentation] Setting normal distance weight to "<< this->normalDistanceWeight;
				this->objectModelUsingNormals->setNormalDistanceWeight(this->normalDistanceWeight);
			}

			if (this->objectModelUsingNormals->getAxis() != this->axis)
			{
				cout<<"[SAC Segmentation] Setting the axis to"<< axis[0] << ", "
						<< axis[1] << ", " <<axis[2] <<endl;
				this->objectModelUsingNormals->setAxis(axis);
			}
			if (this->objectModelUsingNormals->getEpsAngle () != this->epsAngle)
			{
				cout << "[SAC Segmentation] Setting the epsilon angle to "<< this->epsAngle;
				this->objectModelUsingNormals->setEpsAngle (this->epsAngle);
			}
			break;
		}
		default:
		{
			cout<<"[OBJModelPlane : initSACModel] No valid model given!";
			return (false);
		}
		}
		return (true);
	}




	/** @brief Initialize the segmentation method and set the required parameters.
	 *  @param methodType the type of Sample Consensus method to be used
	 */
	inline void initSACMethod(const int methodType) {



		switch (methodType) {
		case SAC_ALMeDS: {
			cout<<"[SAC Segmentation] Using a method of type: SAC_ALMeDS with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodALMeDS();
			sacMethod->setObjectModel(objectModelUsingNormals);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_RANSAC:
		{
			cout<< "[SAC Segmentation] Using a method of type: SAC_RANSAC with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodRANSAC();
			sacMethod->setObjectModel(objectModelUsingNormals);

			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_LMEDS: {
			cout<<"[SAC Segmentation] Using a method of type: SAC_LMeDS with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodLMeDS();
			sacMethod->setObjectModel(objectModelUsingNormals);

			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_MSAC: {
			cout<<"[SAC Segmentation] Using a method of type: SAC_MSAC with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodMSAC();
			sacMethod->setObjectModel(objectModelUsingNormals);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_MLESAC: {
			cout<<"[SAC Segmentation] Using a method of type: SAC_MLESAC with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodMLESAC();
			sacMethod->setObjectModel(objectModelUsingNormals);
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
		initSACMethod(SACMethodType);

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

#endif /* REGIONBASEDSACSEGMENTATIONUSINGNORMALS_H_ */
