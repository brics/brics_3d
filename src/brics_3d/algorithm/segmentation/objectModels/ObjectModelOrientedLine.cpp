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

#include "ObjectModelOrientedLine.h"

namespace brics_3d {

void
  ObjectModelOrientedLine::selectWithinDistance (const Eigen::VectorXd &model_coefficients,
		  double threshold, std::vector<int> &inliers)
{
  // Obtain the line direction
  Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4],
		  model_coefficients[5], 0);

  // Check against template, if given
  if (epsAngle > 0.0)
  {
    double angleDifference = fabs (getAngle3D (axis, line_dir));
    angleDifference = fmin (angleDifference, M_PI - angleDifference);
    // Check whether the current plane model satisfies our angle threshold
    //criterion with respect to the given axis
    if (angleDifference > epsAngle)
    {
      inliers.resize (0);
      return;
    }
  }

  this->selectWithinDistance (model_coefficients, threshold, inliers);
}

void
   ObjectModelOrientedLine::getDistancesToModel (const Eigen::VectorXd &model_coefficients,
		   std::vector<double> &distances)
 {
   // Obtain the line direction
   Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4],
		   model_coefficients[5], 0);

   // Check against template, if given
   if (epsAngle > 0.0)
   {
     double angleDifference = fabs (getAngle3D (axis, line_dir));
     angleDifference = fmin (angleDifference, M_PI - angleDifference);
     // Check whether the current plane model satisfies our angle threshold
     //criterion with respect to the given axis
     if (angleDifference > epsAngle)
     {
       distances.resize (0);
       return;
     }
   }

   this->getDistancesToModel (model_coefficients, distances);
 }

}
