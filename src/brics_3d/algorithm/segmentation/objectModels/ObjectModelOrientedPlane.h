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
#ifndef BRICS_3D_OBJECTMODELORIENTEDPLANE_H_
#define BRICS_3D_OBJECTMODELORIENTEDPLANE_H_

#include "algorithm/segmentation/objectModels/ObjectModelPlane.h"

namespace brics_3d {

/**
 * @note The implementation is reusing the object model implementation in ROS:PCl
 * @ingroup segmentation
 */
class ObjectModelOrientedPlane : public ObjectModelPlane {

private:

	/** @brief The axis along which we need to search for a plane perpendicular to. */
    Eigen::Vector4d axis;

	/** @brief The maximum angle between the model normal and the given axis */
	double epsAngle;

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

#endif /* BRICS_3D_OBJECTMODELORIENTEDPLANE_H_ */
