/*
 * RegionBasedSACSegmentationUsingNormals.h
 *
 *  Created on: Apr 24, 2011
 *      Author: reon
 */

#ifndef REGIONBASEDSACSEGMENTATIONUSINGNORMALS_H_
#define REGIONBASEDSACSEGMENTATIONUSINGNORMALS_H_
#include <vector>
#include <Eigen/Geometry>


//**********************************************INFO***************************//
//This is not a working copy. Will be updated once normal extraction is done.


#include "core/NormalSet3D.h"
#include "RegionBasedSACSegmentation.h"



//Object Models Supported
#include "algorithm/segmentation/objectModels/ObjectModelCylinder.h"
#include "algorithm/segmentation/objectModels/ObjectModelNormalPlane.h"


namespace BRICS_3D {

class RegionBasedSACSegmentationUsingNormals : public RegionBasedSACSegmentation{

private:

	/** \brief The minimum and maximum radius limits for the model. Applicable to all models that estimate a radius. */
		double radiusMin, radiusMax;

	/** \brief The relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal */
	double normalDistanceWeight;

	/** \brief A pointer to vector of normals */
	NormalSet3D *normalSet;

	/** \brief Axis along which we need to search for a model perpendicular to */
	Eigen::Vector3d axis;

	/** \brief Set the angle epsilon (delta) threshold */
	double epsAngle;

public:
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


    /** \brief Provide a pointer to the input dataset that contains the point normals of the XYZ dataset.
      * \param normals the const boost shared pointer to a PointCloud message
      */
    inline void
      setInputNormals (NormalSet3D *normalSet)
    {
      this->normalSet = normalSet;
    }


    /** \brief Get a pointer to the normals of the input XYZ point cloud dataset. */
    inline NormalSet3D*
      getInputNormals ()
    {
      return (this->normalSet);
    }


    /** \brief Set the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point
      * normals and the plane normal.
      * \param distance_weight the distance/angular weight
      */
    inline void
      setNormalDistanceWeight (double distanceWeight)
    {
      this->normalDistanceWeight = distanceWeight;
    }


    /** \brief Get the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point
      * normals and the plane normal. */
    inline double
      getNormalDistanceWeight ()
    {
      return (this->normalDistanceWeight);
    }


    /** \brief Set the axis along which we need to search for a model perpendicular to.
      * \param ax the axis along which we need to search for a model perpendicular to
      */
    inline void
      setAxis (const Eigen::Vector3d &ax)
    {
      this->axis = ax;
    }


    /** \brief Get the axis along which we need to search for a model perpendicular to. */
    inline Eigen::Vector3d
      getAxis ()
    {
      return (this->axis);
    }


    /** \brief Set the angle epsilon (delta) threshold.
      * \param ea the maximum allowed difference between the model normal and the given axis.
      */
    inline void
      setEpsAngle (double ea)
    {
      this->epsAngle = ea;
    }


    /** \brief Get the epsilon (delta) model angle threshold. */
    inline double
      getEpsAngle ()
    {
      return (this->epsAngle);
    }



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



    /** \brief Initialize the Sample Consensus model and set its parameters.
      * \param model_type the type of SAC model that is to be used
      */
virtual    inline bool
      initSACModel (const int model_type)
    {
      // Check if input is synced with the normals
      if (this->inputPointCloud->getSize() != this->normalSet->getSize())
      {
        cout<<"[initSACModelUsing Normals] The number of points inthe input point cloud differs than the number of points in the normals!";
        return (false);
      }

      // Build the model
      switch (model_type)
      {
        case OBJMODEL_CYLINDER:
        {
          cout<<"[OBJModelCylinder : initSACModel] Using a model of type: OBJMODEL_CYLINDER"<<endl;

          this->objectModelUsingNormals =  new ObjectModelCylinder();
          this->objectModelUsingNormals->setInputCloud(this->inputPointCloud);
          this->objectModelUsingNormals->setInputNormals(this->normalSet);

          // Set the input normals
          double min_radius, max_radius;
          this->objectModelUsingNormals->getRadiusLimits(min_radius,max_radius);
          if (this->radiusMin != min_radius && this->radiusMax!= max_radius)
          {
            cout<<"[OBJModelCylinder : initSACModel] Setting radius limits to "<< this->radiusMin<< ", "<<this->radiusMax<<endl;
            this->objectModelUsingNormals->setRadiusLimits (this->radiusMin, this->radiusMax);
          }
          if (this->normalDistanceWeight != this->objectModelUsingNormals->getNormalDistanceWeight ())
          {
            cout<< "[OBJModelCylinder : initSACModel] Setting normal distance weight to "<< this->normalDistanceWeight <<endl;
            this->objectModelUsingNormals->setNormalDistanceWeight (this->normalDistanceWeight);
          }
          break;
        }
        case OBJMODEL_NORMAL_PLANE:
        {
          cout<<"[OBJModelNormalPlane : initSACModel] Using a model of type: OBJMODEL_NORMAL_PLANE"<<endl;
          this->objectModelUsingNormals = new ObjectModelNormalPlane();

          // Set the input normals
          this->objectModelUsingNormals->setInputNormals(this->normalSet);

          if (normalDistanceWeight != this->objectModelUsingNormals->getNormalDistanceWeight ())
          {
            cout<< "[OBJModelNormalPlane : initSACModel] Setting normal distance weight to "<< this->normalDistanceWeight;
            this->objectModelUsingNormals->setNormalDistanceWeight(this->normalDistanceWeight);
          }

          if (this->objectModelUsingNormals->getAxis() != this->axis)
          {
            cout<<"[OBJModelNormalPlane : initSACModel] Setting the axis to"<< axis[0] << ", "
            		<< axis[1] << ", " <<axis[2] <<endl;
            this->objectModelUsingNormals->setAxis(axis);
          }
          if (this->objectModelUsingNormals->getEpsAngle () != this->epsAngle)
          {
            cout << "[OBJModelPlane : initSACModel] Setting the epsilon angle to "<< this->epsAngle;
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




	/** \brief Initialize the Sample Consensus method and set its parameters.
	 * \param method_type the type of SAC method to be used
	 */
	virtual inline void initSACMethod(const int method_type) {
		// Build the sample consensus method
		switch (method_type) {
		case SAC_ALMeDS: {
			cout<<"[initSAC] Using a method of type: SAC_ALMeDS with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodALMeDS();
			sacMethod->setObjectModel(objectModelUsingNormals);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_RANSAC:
		{
			cout<< "[initSAC] Using a method of type: SAC_RANSAC_ROS with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodRANSAC_ROS();
			sacMethod->setObjectModel(objectModelUsingNormals);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_LMEDS: {
			cout<<"[initSAC] Using a method of type: SAC_LMeDS_ROS with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodLMeDS_ROS();
			sacMethod->setObjectModel(objectModelUsingNormals);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_MSAC: {
			cout<<"[initSAC] Using a method of type: SAC_MSAC_ROS with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodMSAC_ROS();
			sacMethod->setObjectModel(objectModelUsingNormals);
			sacMethod->setDistanceThreshold(threshold);
			sacMethod->setPointCloud(inputPointCloud);
			break;
		}
		case SAC_MLESAC: {
			cout<<"[initSAC] Using a method of type: SAC_MLESAC_ROS with a model threshold of "<<threshold<<endl;
			sacMethod = new SACMethodMLESAC_ROS();
			sacMethod->setObjectModel(objectModelUsingNormals);
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
	segment (std::vector<int> &inliers, Eigen::VectorXd &model_coefficients)
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
		Eigen::VectorXd coeff;
		sacMethod->getModelCoefficients (coeff);


		// If the user needs optimized coefficients
		if (optimizeCoefficients)
		{
			Eigen::VectorXd coeff_refined;

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

#endif /* REGIONBASEDSACSEGMENTATIONUSINGNORMALS_H_ */
