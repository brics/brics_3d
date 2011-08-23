/*
 * @file:ObjectModelOrientedPlane.cpp
 *
 * @date:Created on: Apr 22, 2011
 * @author:Author: reon
 * @note The implementation is reusing the object model implementation in ROS:PCl
 */


#include "ObjectModelOrientedPlane.h"

namespace BRICS_3D {

void ObjectModelOrientedPlane::selectWithinDistance(const Eigen::VectorXd &model_coefficients, double threshold,
			std::vector<int> &inliers){

     // Obtain the plane normal
     Eigen::Vector4d coeff = model_coefficients;
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


void ObjectModelOrientedPlane::getDistancesToModel (const Eigen::VectorXd &model_coefficients,
		std::vector<double> &distances){
    //ToDo check for (model_coefficients.size () == 4);

    // Obtain the plane normal
    Eigen::Vector4d coeff = model_coefficients;
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
