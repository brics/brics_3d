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



#include "RegionBasedSACSegmentation.h"



//Object Models Supported
#include "algorithm/segmentation/objectModels/ObjectModelCylinder.h"
#include "algorithm/segmentation/objectModels/ObjectModelNormalPlane.h"

namespace BRICS_3D {

typedef double pointNormal[3];
typedef std::vector<pointNormal> pointCloudNormals;

class RegionBasedSACSegmentationUsingNormals : public RegionBasedSACSegmentation{

private:

	/** \brief The minimum and maximum radius limits for the model. Applicable to all models that estimate a radius. */
		double radiusMin, radiusMax;

	/** \brief The relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal */
	double normalDistanceWeight;

	/** \brief A pointer to vector of normals */
	pointCloudNormals *normals;

	/** \brief Axis along which we need to search for a model perpendicular to */
	Eigen::Vector3f axis;

	/** \brief Set the angle epsilon (delta) threshold */
	double epsAngle;

public:
	RegionBasedSACSegmentationUsingNormals(){};
	virtual ~RegionBasedSACSegmentationUsingNormals();


    /** \brief Provide a pointer to the input dataset that contains the point normals of the XYZ dataset.
      * \param normals the const boost shared pointer to a PointCloud message
      */
    inline void
      setInputNormals (pointCloudNormals *normals)
    {
      this->normals = normals;
    }


    /** \brief Get a pointer to the normals of the input XYZ point cloud dataset. */
    inline pointCloudNormals*
      getInputNormals ()
    {
      return (this->normals);
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
      setAxis (const Eigen::Vector3f &ax)
    {
      this->axis = ax;
    }


    /** \brief Get the axis along which we need to search for a model perpendicular to. */
    inline Eigen::Vector3f
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
    virtual inline bool
      initSACModel (const int model_type)
    {
      // Check if input is synced with the normals
      if (this->inputPointCloud->getSize() != this->normals->size())
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
          this->objectModelUsingNormals->setInputNormals(this->normals);

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
          this->objectModelUsingNormals->setInputNormals(this->normals);

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

};

}

#endif /* REGIONBASEDSACSEGMENTATIONUSINGNORMALS_H_ */
