/*
 * ObjectModelOrientedPlane.h
 *
 *  Created on: Apr 22, 2011
 *      Author: reon
 */

#ifndef OBJECTMODELORIENTEDPLANE_H_
#define OBJECTMODELORIENTEDPLANE_H_

#include "algorithm/segmentation/objectModels/ObjectModelPlane.h"

namespace BRICS_3D {

class ObjectModelOrientedPlane : public ObjectModelPlane {

private:

	/** \brief The axis along which we need to search for a plane perpendicular to. */
    Eigen::Vector4d axis;

    /** \brief The maximum allowed difference between the plane normal and the given axis. */
    double epsAngle;

public:
	ObjectModelOrientedPlane(){};
	virtual ~ObjectModelOrientedPlane(){};


    /** \brief Set the axis along which we need to search for a plane perpendicular to.
      * \param ax the axis along which we need to search for a plane perpendicular to
      */
    inline void setAxis (const Eigen::Vector3d &ax)
    {
      axis.start<3> () = ax;
      axis[3] = 0;
    }


    /** \brief Get the axis along which we need to search for a plane perpendicular to. */
    inline Eigen::Vector3d getAxis ()  { return (axis.start<3> ()); }

    /** \brief Set the angle epsilon (delta) threshold.
      * \param ea the maximum allowed difference between the plane normal and the given axis.
      */
    inline void setEpsAngle (double ea) { epsAngle = ea; }


    /** \brief Get the angle epsilon (delta) threshold. */
    inline double getEpsAngle () { return (epsAngle); }

    void selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold,
    			std::vector<int> &inliers);
    void getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances);

};
}

#endif /* OBJECTMODELORIENTEDPLANE_H_ */
