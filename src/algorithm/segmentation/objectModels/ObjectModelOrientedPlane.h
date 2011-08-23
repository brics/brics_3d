/*
 * @file:ObjectModelOrientedPlane.h
 *
 * @date:Created on: Apr 22, 2011
 * @author:Author: reon
 * @note The implementation is reusing the object model implementation in ROS:PCl
 */

#ifndef OBJECTMODELORIENTEDPLANE_H_
#define OBJECTMODELORIENTEDPLANE_H_

#include "algorithm/segmentation/objectModels/ObjectModelPlane.h"

namespace BRICS_3D {

class ObjectModelOrientedPlane : public ObjectModelPlane {

private:

	/** @brief The axis along which we need to search for a plane perpendicular to. */
    Eigen::Vector4d axis;

	/** @brief The maximum angle between the model normal and the given axis */
	double epsAngle;

public:
	ObjectModelOrientedPlane(){};
	virtual ~ObjectModelOrientedPlane(){};


    /** @brief Set the axis along which we need to search for a plane perpendicular to.
      * @param ax the axis along which we need to search for a plane perpendicular to
      */
    inline void setAxis (const Eigen::Vector3d &ax)
    {

#ifdef EIGEN3
		axis.head<3> () = ax;
#else
		axis.start<3> () = ax;
#endif
      axis[3] = 0;
    }


    /** @brief Get the axis along which we need to search for a plane perpendicular to. */
    inline Eigen::Vector3d getAxis ()  {
#ifdef EIGEN3
    	return (axis.head<3> ());
#else
		return (axis.start<3> ());
#endif
    }

    /** @brief Set the angle epsilon (delta) threshold.
      * @param ea the maximum allowed difference between the plane normal and the given axis.
      */
    inline void setEpsAngle (double ea) { epsAngle = ea; }


    /** @brief Get the angle epsilon (delta) threshold. */
    inline double getEpsAngle () { return (epsAngle); }

    void selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold,
    			std::vector<int> &inliers);
    void getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances);

};
}

#endif /* OBJECTMODELORIENTEDPLANE_H_ */
