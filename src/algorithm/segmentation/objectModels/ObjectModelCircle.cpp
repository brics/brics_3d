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

#include "ObjectModelCircle.h"


namespace BRICS_3D {


void ObjectModelCircle::computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate,
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

void
ObjectModelCircle::getSamples (int &iterations, std::vector<int> &samples)
{

	samples.resize (3);
	double trand = this->inputPointCloud->getSize() / (RAND_MAX + 1.0);

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

	this->points = inputPointCloud->getPointCloud();

	// Get the values at the two points
	Eigen::Vector2d p0 = Eigen::Vector2d (this->points->data()[samples[0]].getX(), this->points->data()[samples[0]].getY());
	Eigen::Vector2d p1 = Eigen::Vector2d (this->points->data()[samples[1]].getX(), this->points->data()[samples[1]].getY());

	// Compute the segment values (in 2d) between p1 and p0
	p1 -= p0;

	Eigen::Vector2d dy1dy2;
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

		Eigen::Vector2d p2 = Eigen::Vector2d (this->points->data()[samples[2]].getX(), this->points->data()[samples[2]].getY());

		// Compute the segment values (in 2d) between p2 and p0
		p2 -= p0;

#ifdef EIGEN3
		dy1dy2 = p1.array () / p2.array();
#else
		dy1dy2 = p1.cwise () / p2;
#endif
		++iter;
		if (iter > MAX_ITERATIONS_COLLINEAR )
		{
			cout<<"[SampleConsensusModelCircle::getSamples] WARNING: Could not select "
					"3 non collinear points in"<<
					MAX_ITERATIONS_COLLINEAR <<" iterations!";
			break;
		}
		iterations++;
	}
	while ( (dy1dy2[0] == dy1dy2[1]) );
	iterations--;
}

bool
ObjectModelCircle::computeModelCoefficients (const std::vector<int> &samples, Eigen::VectorXd &model_coefficients)
{
	//ToDo Check for (samples.size () == 3);

	model_coefficients.resize (3);

	Eigen::Vector2d p0 (this->points->data()[samples[0]].getX(), this->points->data()[samples[0]].getY());
	Eigen::Vector2d p1 (this->points->data()[samples[1]].getX(), this->points->data()[samples[1]].getY());
	Eigen::Vector2d p2 (this->points->data()[samples[2]].getX(), this->points->data()[samples[2]].getY());

	Eigen::Vector2d u = (p0 + p1) / 2.0;
	Eigen::Vector2d v = (p1 + p2) / 2.0;

	Eigen::Vector2d p1p0dif = p1 - p0;
	Eigen::Vector2d p2p1dif = p2 - p1;
	Eigen::Vector2d uvdif   = u - v;

	Eigen::Vector2d m = Eigen::Vector2d (- p1p0dif[0] / p1p0dif[1], - p2p1dif[0] / p2p1dif[1]);

	// Center (x, y)
	model_coefficients[0] = (m[0] * u[0] -  m[1] * v[0]  - uvdif[1] )             / (m[0] - m[1]);
	model_coefficients[1] = (m[0] * m[1] * uvdif[0] +  m[0] * v[1] - m[1] * u[1]) / (m[0] - m[1]);

	// Radius
	model_coefficients[2] = sqrt ((model_coefficients[0] - p0[0]) * (model_coefficients[0] - p0[0]) +
			(model_coefficients[1] - p0[1]) * (model_coefficients[1] - p0[1]));
	return (true);
}

/** @brief Compute all distances from the cloud data to a given 2D circle model.
 * @param model_coefficients the coefficients of a 2D circle model that we need to compute distances to
 * @param distances the resultant estimated distances
 */
void
ObjectModelCircle:: getDistancesToModel (const Eigen::VectorXd &model_coefficients, std::vector<double> &distances)
{
	//ToDo Check for (model_coefficients.size () == 3);

	distances.resize (this->inputPointCloud->getSize());

	// Iterate through the 3d points and calculate the distances from them to the circle
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
		// Calculate the distance from the point to the circle as the difference between
		// dist(point,circle_origin) and circle_radius
		distances[i] = fabs (sqrt (
				( this->points->data()[i].getX() - model_coefficients[0] ) *
				( this->points->data()[i].getX() - model_coefficients[0] ) +

				( this->points->data()[i].getY() - model_coefficients[1] ) *
				( this->points->data()[i].getY() - model_coefficients[1] )
		) - model_coefficients[2]);
}


void
ObjectModelCircle::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
		std::vector<double> &distances) {

	//Todo Check for (model_coefficients.size () == 3);

	distances.resize (inliers.size());

	for (size_t i = 0; i < inliers.size (); ++i)
	{
		// Calculate the distance from the point to the circle as the difference between
		// dist(point,circle_origin) and circle_radius
		distances[i]=fabs (sqrt (
				( this->points->data()[inliers[i]].getX() - model_coefficients[0] ) *
				( this->points->data()[inliers[i]].getX() - model_coefficients[0] ) +

				( this->points->data()[inliers[i]].getY() - model_coefficients[1] ) *
				( this->points->data()[inliers[i]].getY() - model_coefficients[1] )
		) - model_coefficients[2]);
	}
}

void
ObjectModelCircle::selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold, std::vector<int> &inliers)
{

	//Todo Check for (model_coefficients.size () == 3);

	int nr_p = 0;
	inliers.resize (this->inputPointCloud->getSize());

	// Iterate through the 3d points and calculate the distances from them to the sphere
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the sphere as the difference between
		// dist(point,sphere_origin) and sphere_radius
		float distance = fabs (sqrt (
				( this->points->data()[i].getX() - model_coefficients[0] ) *
				( this->points->data()[i].getX() - model_coefficients[0] ) +

				( this->points->data()[i].getY() - model_coefficients[1] ) *
				( this->points->data()[i].getY() - model_coefficients[1] )
		) - model_coefficients[2]);
		if (distance < threshold)
		{
			// Returns the indices of the points whose distances are smaller than the threshold
			inliers[nr_p] = i;
			nr_p++;
		}
	}
	inliers.resize (nr_p);
}

bool
ObjectModelCircle:: doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients, double threshold)
{
	//ToDo Check for (model_coefficients.size () == 3);

	for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
		// Calculate the distance from the point to the sphere as the difference between
		//dist(point,sphere_origin) and sphere_radius
		if (fabs (sqrt (
				( this->points->data()[*it].getX() - model_coefficients[0] ) *
				( this->points->data()[*it].getX() - model_coefficients[0] ) +
				( this->points->data()[*it].getY() - model_coefficients[1] ) *
				( this->points->data()[*it].getY() - model_coefficients[1] )
		) - model_coefficients[2]) > threshold)
			return (false);

	return (true);
}


}

