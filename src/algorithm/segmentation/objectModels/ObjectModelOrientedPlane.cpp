/*
 * ObjectModelOrientedPlane.cpp
 *
 *  Created on: Apr 22, 2011
 *      Author: reon
 */

#include "ObjectModelOrientedPlane.h"

namespace BRICS_3D {

void ObjectModelOrientedPlane::selectWithinDistance(const Eigen::VectorXf &model_coefficients, double threshold,
			std::vector<int> &inliers){
    // Needs a valid set of model coefficients
    //ToDo ROS_ASSERT (model_coefficients.size () == 4);

     // Obtain the plane normal
     Eigen::Vector4f coeff = model_coefficients;
     coeff[3] = 0;

     // Check against template, if given
     if (epsAngle > 0.0)
     {
       double angleDifference = fabs (getAngle3D (axis, coeff));
       angleDifference = fmin (angleDifference, M_PI - angleDifference);
       // Check whether the current plane model satisfies our angle threshold criterion with
       //respect to the given axis
       if (angleDifference > epsAngle)
       {
         inliers.resize (0);
         return;
       }
     }

     this->selectWithinDistance (model_coefficients, threshold, inliers);
}

void ObjectModelOrientedPlane::getDistancesToModel (const Eigen::VectorXf &model_coefficients,
		std::vector<double> &distances){
    // Needs a valid set of model coefficients
    //ToDo ROS_ASSERT (model_coefficients.size () == 4);

    // Obtain the plane normal
    Eigen::Vector4f coeff = model_coefficients;
    coeff[3] = 0;

    // Check against template, if given
    if (epsAngle > 0.0)
    {
      double angleDifference = fabs (getAngle3D (axis, coeff));
      angleDifference = fmin (angleDifference, M_PI - angleDifference);
      // Check whether the current plane model satisfies our angle threshold criterion with respect to the given axis
      if (angleDifference > epsAngle)
      {
        distances.resize (0);
        return;
      }
    }

    this->getDistancesToModel (model_coefficients, distances);
}

}
