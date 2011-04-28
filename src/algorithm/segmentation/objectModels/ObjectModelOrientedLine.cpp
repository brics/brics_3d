/*
 * ObjectModelOrientedLine.cpp
 *
 *  Created on: Apr 23, 2011
 *      Author: reon
 */

#include "ObjectModelOrientedLine.h"

namespace BRICS_3D {

void
  ObjectModelOrientedLine::selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold, std::vector<int> &inliers)
{
  // Obtain the line direction
  Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

  // Check against template, if given
  if (epsAngle > 0.0)
  {
    double angleDifference = fabs (getAngle3D (axis, line_dir));
    angleDifference = fmin (angleDifference, M_PI - angleDifference);
    // Check whether the current plane model satisfies our angle threshold criterion with respect to the given axis
    if (angleDifference > epsAngle)
    {
      inliers.resize (0);
      return;
    }
  }

  this->selectWithinDistance (model_coefficients, threshold, inliers);
}

void
   ObjectModelOrientedLine::getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances)
 {
   // Obtain the line direction
   Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

   // Check against template, if given
   if (epsAngle > 0.0)
   {
     double angleDifference = fabs (getAngle3D (axis, line_dir));
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
