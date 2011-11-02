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
#include "ObjectModelPlane.h"


namespace BRICS_3D {

void ObjectModelPlane::computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate,
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

void ObjectModelPlane::getSamples(int &iterations, std::vector<int> &samples){

	points = inputPointCloud->getPointCloud();

	samples.resize (3);
	double trand = inputPointCloud->getSize() / (RAND_MAX + 1.0);

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
	p1 = Eigen::Vector4d (this->points->data()[samples[1]].getX(), this->points->data()[samples[1]].getY(), this->points->data()[samples[1]].getZ(), 0);
	p0 = Eigen::Vector4d (this->points->data()[samples[0]].getX(), this->points->data()[samples[0]].getY(), this->points->data()[samples[0]].getZ(), 0);

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
		p2 = Eigen::Vector4d (this->points->data()[samples[2]].getX(), this->points->data()[samples[2]].getY(), this->points->data()[samples[2]].getZ(), 0);

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
			cout<<"[SampleConsensusModelPlane::getSamples] WARNING: Could not select "
					"3 collinear points in"
					<<  MAX_ITERATIONS_COLLINEAR <<"iterations!";
			break;
		}
		iterations++;
	}
	while ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) );
	iterations--;

}

bool ObjectModelPlane::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXd &model_coefficients){
	//ToDo Check (samples.size () == 3);

	Eigen::Vector4d p0, p1, p2;
	// SSE friendly data check
	p0 = Eigen::Vector4d (this->points->data()[samples[0]].getX(), this->points->data()[samples[0]].getY(), this->points->data()[samples[0]].getZ(), 0);
	p1 = Eigen::Vector4d (this->points->data()[samples[1]].getX(), this->points->data()[samples[1]].getY(), this->points->data()[samples[1]].getZ(), 0);
	p2 = Eigen::Vector4d (this->points->data()[samples[2]].getX(), this->points->data()[samples[2]].getY(), this->points->data()[samples[2]].getZ(), 0);

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
ObjectModelPlane::getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances)
{
	//Todo Check for a valid set of model coefficients

	distances.resize (this->inputPointCloud->getSize());

	// Iterate through the 3d points and calculate the distances from them to the plane
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the plane normal as the dot product
		// D = (P-A).N/|N|
		distances[i] = fabs (model_coefficients[0] * this->points->data()[i].getX() +
				model_coefficients[1] * this->points->data()[i].getY() +
				model_coefficients[2] * this->points->data()[i].getZ() +
				model_coefficients[3]);
	}
}


void
ObjectModelPlane::selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold, std::vector<int> &inliers)
{
	//Todo Check for a valid set of model coefficients

	int nr_p = 0;
	inliers.resize (this->inputPointCloud->getSize());

	// Iterate through the 3d points and calculate the distances from them to the plane
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the plane normal as the dot product
		// D = (P-A).N/|N|
		if (fabs (model_coefficients[0] * this->points->data()[i].getX() +
				model_coefficients[1] * this->points->data()[i].getY() +
				model_coefficients[2] * this->points->data()[i].getZ() +
				model_coefficients[3]) < threshold)
		{
			// Returns the indices of the points whose distances are smaller than the threshold
			inliers[nr_p] = i;
			nr_p++;
		}
	}
	inliers.resize (nr_p);
}


void
ObjectModelPlane::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,  std::vector<double> &distances) {
	//Todo Check for a valid set of model coefficients

	distances.resize (inliers.size());

	for (size_t i = 0; i < inliers.size (); ++i)
	{
		// Calculate the distance from the point to the plane normal as the dot product
		// D = (P-A).N/|N|
		distances[i]=fabs (model_coefficients[0] * this->points->data()[inliers[i]].getX() +
				model_coefficients[1] * this->points->data()[inliers[i]].getY() +
				model_coefficients[2] * this->points->data()[inliers[i]].getZ() +
				model_coefficients[3]);
	}
}


bool
ObjectModelPlane::doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients, double threshold)
{
	//Todo Check for a valid set of model coefficients

	for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
		if (fabs (model_coefficients[0] * this->points->data()[*it].getX() +
				model_coefficients[1] * this->points->data()[*it].getY() +
				model_coefficients[2] * this->points->data()[*it].getZ() +
				model_coefficients[3]) > threshold)
			return (false);

	return (true);
}


}
