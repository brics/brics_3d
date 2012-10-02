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

#include "ObjectModelNormalPlane.h"

namespace brics_3d {

void ObjectModelNormalPlane::computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate,
		bool &modelFound){

	std::vector<int> samples;
	std::vector<int> selection;
	getSamples(iterations,selection);

	if (selection.size () == 0){
		isDegenerate = false;
		modelFound = false;
		return;
	} else {

		isDegenerate = true;

	}

	if (!computeModelCoefficients (selection, model_coefficients)){
		modelFound = false;
		return;
	} else {
		modelFound = true;
		return;
	}
}

void ObjectModelNormalPlane::getSamples(int &iterations, std::vector<int> &samples){
//	points = inputPointCloud->getPointCloud();


	samples.resize (3);
	double trand = inputPointCloud->getSize() / (RAND_MAX + 1.0);

	// Get a random number between 1 and max_indices
	int idx = (int)(rand () * trand);
	// Get the index
	samples[0] = idx;

	// Get a second point which is different than the first
	do
	{
		idx = (int)(rand () * trand);
		samples[1] = idx;
		iterations++;
	} while (samples[1] == samples[0]);
	iterations--;
	// Get the values at the two points
	Eigen::Vector4d p0, p1, p2;
	// SSE friendly data check
	p1 = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[samples[1]].getX(), (*inputPointCloud->getPointCloud())[samples[1]].getY(), (*inputPointCloud->getPointCloud())[samples[1]].getZ(), 0);
	p0 = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[samples[0]].getX(), (*inputPointCloud->getPointCloud())[samples[0]].getY(), (*inputPointCloud->getPointCloud())[samples[0]].getZ(), 0);

	// Compute the segment values (in 3d) between p1 and p0
	p1 -= p0;

	Eigen::Vector4d dy1dy2;
	int iter = 0;
	do
	{
		// Get the third point, different from the first two
		do
		{
			idx = (int)(rand () * trand);
			samples[2] = idx;
			iterations++;
		} while ( (samples[2] == samples[1]) || (samples[2] == samples[0]) );
		iterations--;

		// SSE friendly data check
		p2 = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[samples[2]].getX(), (*inputPointCloud->getPointCloud())[samples[2]].getY(), (*inputPointCloud->getPointCloud())[samples[2]].getZ(), 0);

		// Compute the segment values (in 3d) between p2 and p0
		p2 -= p0;

#ifdef EIGEN3
		dy1dy2 = p1.array () / p2.array();
#else
		dy1dy2 = p1.cwise () / p2;
#endif
		++iter;
		if (iter > MAX_ITERATIONS_COLLINEAR )
		{
			cout<<"[SampleConsensusModelNormalPlane::getSamples] WARNING: Could not select "
					"3 collinear points in"
					<<  MAX_ITERATIONS_COLLINEAR <<"iterations!";
			break;
		}
		iterations++;
	}
	while ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) );
	iterations--;

}


bool ObjectModelNormalPlane::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXd &model_coefficients){

	assert (samples.size () == 3);

	Eigen::Vector4d p0, p1, p2;
	// SSE friendly data check
	p0 = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[samples[0]].getX(), (*inputPointCloud->getPointCloud())[samples[0]].getY(), (*inputPointCloud->getPointCloud())[samples[0]].getZ(), 0);
	p1 = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[samples[1]].getX(), (*inputPointCloud->getPointCloud())[samples[1]].getY(), (*inputPointCloud->getPointCloud())[samples[1]].getZ(), 0);
	p2 = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[samples[2]].getX(), (*inputPointCloud->getPointCloud())[samples[2]].getY(), (*inputPointCloud->getPointCloud())[samples[2]].getZ(), 0);

	// Compute the segment values (in 3d) between p1 and p0
	p1 -= p0;
	// Compute the segment values (in 3d) between p2 and p0
	p2 -= p0;

	// Avoid some crashes by checking for collinearity here
#ifdef EIGEN3
	Eigen::Vector4d dy1dy2 = p1.array () / p2.array();
#else
	Eigen::Vector4d dy1dy2 = p1.cwise () / p2;
#endif
	if ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) )          // Check for collinearity
		return (false);

	// Compute the plane coefficients from the 3 given points in a straightforward manner
	// calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
	model_coefficients.resize (4);
	model_coefficients[0] = p1[1] * p2[2] - p1[2] * p2[1];
	model_coefficients[1] = p1[2] * p2[0] - p1[0] * p2[2];
	model_coefficients[2] = p1[0] * p2[1] - p1[1] * p2[0];
	model_coefficients[3] = 0;

	// Normalize
	model_coefficients.normalize ();

	// ... + d = 0
	model_coefficients[3] = -1 * (model_coefficients.dot (p0));

	return (true);
}


void
ObjectModelNormalPlane::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,  std::vector<double> &distances) {
	// Needs a valid model coefficients
	assert(model_coefficients.size () == 4);

	distances.resize (inliers.size());

	for (size_t i = 0; i < inliers.size (); ++i)
	{
		// Calculate the distance from the point to the plane normal as the dot product
		// D = (P-A).N/|N|
		distances[i]=fabs (model_coefficients[0] * (*inputPointCloud->getPointCloud())[inliers[i]].getX() +
				model_coefficients[1] * (*inputPointCloud->getPointCloud())[inliers[i]].getY() +
				model_coefficients[2] * (*inputPointCloud->getPointCloud())[inliers[i]].getZ() +
				model_coefficients[3]);
	}
}


bool
ObjectModelNormalPlane::doSamplesVerifyModel (const std::set<int> &indices,
		const Eigen::VectorXd &model_coefficients, double threshold)
{
	assert (model_coefficients.size () == 4);

	for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
		if (fabs (model_coefficients[0] * (*inputPointCloud->getPointCloud())[*it].getX() +
				model_coefficients[1] * (*inputPointCloud->getPointCloud())[*it].getY() +
				model_coefficients[2] * (*inputPointCloud->getPointCloud())[*it].getZ() +
				model_coefficients[3]) > threshold)
			return (false);

	return (true);
}


void
  ObjectModelNormalPlane::selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold, std::vector<int> &inliers){

    assert (model_coefficients.size () == 4);
    if (!this->normals && this->normals->getSize()!=this->inputPointCloud->getSize())
    {
      cout<<"[ObjectModelNormalPlane::getDistancesToModel] No input dataset containing normals was given!";
      return;
    }

    Eigen::Vector4d coeff = model_coefficients;
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
      Eigen::Vector4d p = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[i].getX(),
    		  (*inputPointCloud->getPointCloud())[i].getY(), (*inputPointCloud->getPointCloud())[i].getZ(), 0);

      Eigen::Vector4d n = Eigen::Vector4d (this->normals->getNormals()->data()[i].getX(),
    		  this->normals->getNormals()->data()[i].getY(),
    		  this->normals->getNormals()->data()[i].getZ(), 0);

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
  ObjectModelNormalPlane::getDistancesToModel (const Eigen::VectorXd &model_coefficients,
		  std::vector<double> &distances){

    assert (model_coefficients.size () == 4);

    if (!this->normals && this->normals->getSize()!=this->inputPointCloud->getSize())
    {
      cout<<"[ObjectModelNormalPlane::getDistancesToModel] No input dataset containing normals was given!";
      return;
    }

    distances.resize (this->inputPointCloud->getSize());

    Eigen::Vector4d coeff = model_coefficients;
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
      Eigen::Vector4d p = Eigen::Vector4d ((*inputPointCloud->getPointCloud())[i].getX(),
    		  (*inputPointCloud->getPointCloud())[i].getY(), (*inputPointCloud->getPointCloud())[i].getZ(), 0);

      Eigen::Vector4d n = Eigen::Vector4d (this->normals->getNormals()->data()[i].getX(),
    		  this->normals->getNormals()->data()[i].getY(), this->normals->getNormals()->data()[i].getZ(), 0);
      double d_euclid = fabs (coeff.dot (p) + model_coefficients[3]);

      // Calculate the angular distance between the point normal and the plane normal
      double d_normal = fabs (getAngle3D (n, coeff));
      d_normal = fmin (d_normal, M_PI - d_normal);

      distances[i] = fabs (this->normalDistanceWeight * d_normal + (1 - this->normalDistanceWeight) * d_euclid);
    }

}


}
