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
