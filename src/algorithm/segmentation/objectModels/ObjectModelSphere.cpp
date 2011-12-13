/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Sebastian Blumenthal
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

#include "ObjectModelSphere.h"

namespace BRICS_3D {


void ObjectModelSphere::computeRandomModel (int &iterations, Eigen::VectorXd &modelCoefficients,
		bool &isDegenerate, bool &modelFound){

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

	if (!computeModelCoefficients (selection, modelCoefficients)){
		modelFound = false;
		return;
	} else {
		modelFound = true;
		return;
	}

}

/**
 *  @note gets 4 unique points with at least three of them non- collinear (3 unique points define a circle)
 */
void
ObjectModelSphere:: getSamples (int &iterations, std::vector<int> &samples)
{
	samples.resize (4);
	double trand = this->inputPointCloud->getSize() / (RAND_MAX + 1.0);


	int idx = (int)(rand () * trand);
	// Get the random index
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
	Eigen::Vector4f p0, p1, p2;
	p1 = Eigen::Vector4f (this->points->data()[samples[1]].getX(), this->points->data()[samples[1]].getY(), this->points->data()[samples[1]].getZ(), 0);
	p0 = Eigen::Vector4f (this->points->data()[samples[0]].getX(), this->points->data()[samples[0]].getY(), this->points->data()[samples[0]].getZ(), 0);

	// Compute the segment values (in 3d) between p1 and p0
	p1 -= p0;

	Eigen::Vector4f dy1dy2;
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
		p2 = Eigen::Vector4f (this->points->data()[samples[2]].getX(), this->points->data()[samples[2]].getY(), this->points->data()[samples[2]].getZ(), 0);

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
			cout<<"[SampleConsensusModelSphere::getSamples] WARNING: Could not select 3 non collinear points in "<< MAX_ITERATIONS_COLLINEAR<<" iterations!"<<endl;
			break;
		}
		iterations++;
	}
	while ( (dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]) );
	iterations--;

	// ToDo At least one point should not be in the same plane than the other 3
	do
	{
		samples[3] = (int)(rand () * trand);
		iterations++;
	} while ( (samples[3] == samples[2]) || (samples[3] == samples[1]) || (samples[3] == samples[0]) );
	iterations--;

}



bool
ObjectModelSphere::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXd &model_coefficients)
{
	assert(samples.size() == 4);

	Eigen::Matrix4f temp;
	for (int i = 0; i < 4; i++)
	{
		temp (i, 0) = this->points->data()[samples[i]].getX();
		temp (i, 1) = this->points->data()[samples[i]].getY();
		temp (i, 2) = this->points->data()[samples[i]].getZ();
		temp (i, 3) = 1;
	}
	float m11 = temp.determinant ();
	if (m11 == 0)
		return (false);             // the points don't define a sphere!

	for (int i = 0; i < 4; ++i)
		temp (i, 0) = (this->points->data()[samples[i]].getX()) * (this->points->data()[samples[i]].getX()) +
		(this->points->data()[samples[i]].getY()) * (this->points->data()[samples[i]].getY()) +
		(this->points->data()[samples[i]].getZ()) * (this->points->data()[samples[i]].getZ());
	float m12 = temp.determinant ();

	for (int i = 0; i < 4; ++i)
	{
		temp (i, 1) = temp (i, 0);
		temp (i, 0) = this->points->data()[samples[i]].getX();
	}
	float m13 = temp.determinant ();

	for (int i = 0; i < 4; ++i)
	{
		temp (i, 2) = temp (i, 1);
		temp (i, 1) = this->points->data()[samples[i]].getY();
	}
	float m14 = temp.determinant ();

	for (int i = 0; i < 4; ++i)
	{
		temp (i, 0) = temp (i, 2);
		temp (i, 1) = this->points->data()[samples[i]].getX();
		temp (i, 2) = this->points->data()[samples[i]].getY();
		temp (i, 3) = this->points->data()[samples[i]].getZ();
	}
	float m15 = temp.determinant ();

	// Center (x , y, z)
	model_coefficients.resize (4);
	model_coefficients[0] = 0.5 * m12 / m11;
	model_coefficients[1] = 0.5 * m13 / m11;
	model_coefficients[2] = 0.5 * m14 / m11;
	// Radius
	model_coefficients[3] = sqrt (
			model_coefficients[0] * model_coefficients[0] +
			model_coefficients[1] * model_coefficients[1] +
			model_coefficients[2] * model_coefficients[2] - m15 / m11);

	return (true);
}



void
ObjectModelSphere:: getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances)
{
	assert(model_coefficients.size() == 4);

	distances.resize (this->inputPointCloud->getSize());

	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
		// Calculate the distance from the point to the sphere as the difference between
		//dist(point,sphere_origin) and sphere_radius
		distances[i] = fabs (sqrt (
				( this->points->data()[i].getX() - model_coefficients[0] ) *
				( this->points->data()[i].getX() - model_coefficients[0] ) +

				( this->points->data()[i].getY() - model_coefficients[1] ) *
				( this->points->data()[i].getY() - model_coefficients[1] ) +

				( this->points->data()[i].getZ() - model_coefficients[2] ) *
				( this->points->data()[i].getZ() - model_coefficients[2] )
		) - model_coefficients[3]);
}


void
ObjectModelSphere::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
		std::vector<double> &distances) {

	assert(model_coefficients.size() == 4);

	distances.resize (inliers.size());

	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < inliers.size() ; ++i)
		// Calculate the distance from the point to the sphere as the difference between
		//dist(point,sphere_origin) and sphere_radius
		distances[i] = fabs (sqrt (
				( this->points->data()[inliers[i]].getX() - model_coefficients[0] ) *
				( this->points->data()[inliers[i]].getX() - model_coefficients[0] ) +

				( this->points->data()[inliers[i]].getY() - model_coefficients[1] ) *
				( this->points->data()[inliers[i]].getY() - model_coefficients[1] ) +

				( this->points->data()[inliers[i]].getZ() - model_coefficients[2] ) *
				( this->points->data()[inliers[i]].getZ() - model_coefficients[2] )
		) - model_coefficients[3]);
}



void
ObjectModelSphere::selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold, std::vector<int> &inliers)
{

	assert(model_coefficients.size() == 4);
	int nr_p = 0;
	inliers.resize (this->inputPointCloud->getSize());

	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the sphere as the difference between
		// dist(point,sphere_origin) and sphere_radius
		if (fabs (sqrt (
				( this->points->data()[i].getX() - model_coefficients[0] ) *
				( this->points->data()[i].getX() - model_coefficients[0] ) +

				( this->points->data()[i].getY() - model_coefficients[1] ) *
				( this->points->data()[i].getY() - model_coefficients[1] ) +

				( this->points->data()[i].getZ() - model_coefficients[2] ) *
				( this->points->data()[i].getZ() - model_coefficients[2] )
		) - model_coefficients[3]) < threshold)
		{
			// Returns the indices of the points whose distances are smaller than the threshold
			inliers[nr_p] = i;
			nr_p++;
		}
	}
	inliers.resize (nr_p);
}





bool
ObjectModelSphere::doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients, double threshold)
{
	assert(model_coefficients.size() == 4);

	for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
		// Calculate the distance from the point to the sphere as the difference between
		//dist(point,sphere_origin) and sphere_radius
		if (fabs (sqrt (
				( this->points->data()[*it].getX() - model_coefficients[0] ) *
				( this->points->data()[*it].getX() - model_coefficients[0] ) +
				( this->points->data()[*it].getY() - model_coefficients[1] ) *
				( this->points->data()[*it].getY() - model_coefficients[1] ) +
				( this->points->data()[*it].getZ() - model_coefficients[2] ) *
				( this->points->data()[*it].getZ() - model_coefficients[2] )
		) - model_coefficients[3]) > threshold)
			return (false);

	return (true);
}


}
