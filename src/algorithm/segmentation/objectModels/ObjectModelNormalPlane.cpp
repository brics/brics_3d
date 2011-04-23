/*
 * ObjectModelNormalPlane.cpp
 *
 *  Created on: Apr 23, 2011
 *      Author: reon
 */

#include "ObjectModelNormalPlane.h"

namespace BRICS_3D {

void
  ObjectModelNormalPlane::selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold, std::vector<int> &inliers){

    // Needs a valid set of model coefficients
    //ToDo ROS_ASSERT (model_coefficients.size () == 4);
    if (!this->normals && this->normals->size()!=this->inputPointCloud->getSize())
    {
      cout<<"[ObjectModelNormalPlane::getDistancesToModel] No input dataset containing normals was given!";
      return;
    }

    Eigen::Vector4f coeff = model_coefficients;
    coeff[3] = 0;

    // Check against template, if given
    if (epsAngle > 0.0)
    {
      double angle_diff = fabs (getAngle3D (axis, coeff));
      angle_diff = fmin (angle_diff, M_PI - angle_diff);
      if (angle_diff > epsAngle)
      {
        inliers.resize (0);
        return;
      }
    }
    if (epsDistance > 0.0)
    {
      if (fabs (-model_coefficients[3] - distanceFromOrigin) > epsDistance)
      {
        inliers.resize (0);
        return;
      }
    }

    int nr_p = 0;
    inliers.resize (this->inputPointCloud->getSize());
    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
    {
      // Calculate the distance from the point to the plane normal as the dot product
      // D = (P-A).N/|N|
      Eigen::Vector4f p = Eigen::Vector4f (this->points->data()[i].getX(),
    		  this->points->data()[i].getY(), this->points->data()[i].getZ(), 0);

      Eigen::Vector4f n = Eigen::Vector4f (this->normals->data()[i][0],
    		  this->normals->data()[i][1], this->normals->data()[i][2], 0);

      double d_euclid = fabs (coeff.dot (p) + model_coefficients[3]);

      // Calculate the angular distance between the point normal and the plane normal
      double d_normal = fabs (getAngle3D (n, coeff));
      d_normal = fmin (d_normal, M_PI - d_normal);

      if (fabs (this->normalDistanceWeight * d_normal + (1 - this->normalDistanceWeight) * d_euclid)
    		  < threshold)
      {
        // Returns the indices of the points whose distances are smaller than the threshold
        inliers[nr_p] = i;
        nr_p++;
      }
    }
    inliers.resize (nr_p);

}

void
  ObjectModelNormalPlane::getDistancesToModel (const Eigen::VectorXf &model_coefficients,
		  std::vector<double> &distances){

    // Needs a valid set of model coefficients
    //ToDo ROS_ASSERT (model_coefficients.size () == 4);

    if (!this->normals && this->normals->size()!=this->inputPointCloud->getSize())
    {
      cout<<"[ObjectModelNormalPlane::getDistancesToModel] No input dataset containing normals was given!";
      return;
    }

    distances.resize (this->inputPointCloud->getSize());

    Eigen::Vector4f coeff = model_coefficients;
    coeff[3] = 0;

    // Check against template, if given
    if (epsAngle > 0.0)
    {
      double angle_diff = fabs (getAngle3D (axis, coeff));
      angle_diff = fmin (angle_diff, M_PI - angle_diff);
      if (angle_diff > epsAngle)
      {
        distances.resize (0);
        return;
      }
    }
    if (epsDistance > 0.0)
    {
      if (fabs (-model_coefficients[3] - distanceFromOrigin) > epsDistance)
      {
        distances.resize (0);
        return;
      }
    }

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
    {
      // Calculate the distance from the point to the plane normal as the dot product
      // D = (P-A).N/|N|
      Eigen::Vector4f p = Eigen::Vector4f (this->points->data()[i].getX(),
    		  this->points->data()[i].getY(), this->points->data()[i].getZ(), 0);

      Eigen::Vector4f n = Eigen::Vector4f (this->normals->data()[i][0],
    		  this->normals->data()[i][1], this->normals->data()[i][2], 0);
      double d_euclid = fabs (coeff.dot (p) + model_coefficients[3]);

      // Calculate the angular distance between the point normal and the plane normal
      double d_normal = fabs (getAngle3D (n, coeff));
      d_normal = fmin (d_normal, M_PI - d_normal);

      distances[i] = fabs (this->normalDistanceWeight * d_normal + (1 - this->normalDistanceWeight) * d_euclid);
    }

}


}
