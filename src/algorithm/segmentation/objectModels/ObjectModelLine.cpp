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

#include "ObjectModelLine.h"

namespace brics_3d {

void ObjectModelLine::computeRandomModel (int &iterations, Eigen::VectorXd &model_coefficients, bool &isDegenerate,
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

void ObjectModelLine::getSamples (int &iterations, std::vector<int> &samples){


	samples.resize (2);
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

		if (iterations > MAX_ITERATIONS_COLLINEAR)
		{
			cout<<"[ObjectModelLine::getSamples] WARNING: Could not select 2 unique points in "
					<<MAX_ITERATIONS_COLLINEAR<<" iterations!";
			break;
		}
	} while (samples[1] == samples[0]);
	iterations--;
}

bool ObjectModelLine::computeModelCoefficients (const std::vector<int> &samples,
		Eigen::VectorXd &model_coefficients){

	assert (samples.size () == 2);

	model_coefficients.resize (6);
	model_coefficients[0] = (*inputPointCloud->getPointCloud())[samples[0]].getX();
	model_coefficients[1] = (*inputPointCloud->getPointCloud())[samples[0]].getY();
	model_coefficients[2] = (*inputPointCloud->getPointCloud())[samples[0]].getZ();

	model_coefficients[3] = (*inputPointCloud->getPointCloud())[samples[1]].getX() - model_coefficients[0];
	model_coefficients[4] = (*inputPointCloud->getPointCloud())[samples[1]].getY() - model_coefficients[1];
	model_coefficients[5] = (*inputPointCloud->getPointCloud())[samples[1]].getZ() - model_coefficients[2];

#ifdef EIGEN3
	model_coefficients.tail<3> ().normalize ();
#else
	model_coefficients.end<3> ().normalize ();
#endif
	return (true);
}

void ObjectModelLine::optimizeModelCoefficients (const std::vector<int> &inliers,
		const Eigen::VectorXd &model_coefficients,
		Eigen::VectorXd &optimized_coefficients){
	assert (model_coefficients.size () == 6);

	if (inliers.size () == 0)
	{
		cout<<"[ObjectModelLine::optimizeModelCoefficients] Inliers vector empty! Returning the same coefficients";
		optimized_coefficients = model_coefficients;
		return;
	}

	assert (inliers.size () > 2);

	optimized_coefficients.resize (6);

	// Compute the 3x3 covariance matrix
	Eigen::Vector4d centroid;
	compute3DCentroid (this->inputPointCloud, inliers, centroid);
	Eigen::Matrix3d covariance_matrix;
	computeCovarianceMatrix (inputPointCloud, inliers, centroid, covariance_matrix);
	optimized_coefficients[0] = centroid[0];
	optimized_coefficients[1] = centroid[1];
	optimized_coefficients[2] = centroid[2];

	// Extract the eigenvalues and eigenvectors
	//cloud_geometry::eigen_cov (covariance_matrix, eigen_values, eigen_vectors);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
	EIGEN_ALIGN_MEMORY Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
	EIGEN_ALIGN_MEMORY Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

#ifdef EIGEN3
	optimized_coefficients.tail<3> () = eigen_vectors.col (2).normalized ();
#else
	optimized_coefficients.end<3> () = eigen_vectors.col (2).normalized ();
#endif

}


void ObjectModelLine::getDistancesToModel (const Eigen::VectorXd &model_coefficients,
		std::vector<double> &distances){
	assert (model_coefficients.size () == 6);

	distances.resize (this->inputPointCloud->getSize());

	// Obtain the line point and direction
	Eigen::Vector4d line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	Eigen::Vector4d line_p2 = line_pt + line_dir;

	// Iterate through the 3d points and calculate the distances from them to the line
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the line
		// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
		Eigen::Vector4d pt ((*inputPointCloud->getPointCloud())[i].getX(), (*inputPointCloud->getPointCloud())[i].getY(),
				(*inputPointCloud->getPointCloud())[i].getZ(), 0);
		Eigen::Vector4d pp = line_p2 - pt;

#ifdef EIGEN3
		Eigen::Vector3d c = pp.head<3> ().cross (line_dir.head<3> ());
#else
		Eigen::Vector3d c = pp.start<3> ().cross (line_dir.start<3> ());
#endif
		//distances[i] = sqrt (c.dot (c)) / line_dir.dot (line_dir);
		distances[i] = c.dot (c) / line_dir.dot (line_dir);
	}
}

void ObjectModelLine::selectWithinDistance (const Eigen::VectorXd &model_coefficients, double threshold,
		std::vector<int> &inliers){
	assert (model_coefficients.size () == 6);

	double sqr_threshold = threshold * threshold;

	int nr_p = 0;
	inliers.resize (this->inputPointCloud->getSize());

	// Obtain the line point and direction
	Eigen::Vector4d line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	Eigen::Vector4d line_p2 = line_pt + line_dir;

	// Iterate through the 3d points and calculate the distances from them to the line
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the line
		// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
		Eigen::Vector4d pt ((*inputPointCloud->getPointCloud())[i].getX(),
				(*inputPointCloud->getPointCloud())[i].getY(), (*inputPointCloud->getPointCloud())[i].getZ(), 0);
		Eigen::Vector4d pp = line_p2 - pt;

#ifdef EIGEN3
		Eigen::Vector3d c = pp.head<3> ().cross (line_dir.head<3> ());
#else
		Eigen::Vector3d c = pp.start<3> ().cross (line_dir.start<3> ());
#endif
		//distances[i] = sqrt (c.dot (c)) / line_dir.dot (line_dir);
		double sqr_distance = c.dot (c) / line_dir.dot (line_dir);

		if (sqr_distance < sqr_threshold)
		{
			// Returns the indices of the points whose squared distances are smaller than the threshold
			inliers[nr_p] = i;
			nr_p++;
		}
	}
	inliers.resize (nr_p);
}


void ObjectModelLine::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
		std::vector<double> &distances){

	assert (model_coefficients.size () == 6);

	distances.resize (this->inputPointCloud->getSize());

	// Obtain the line point and direction
	Eigen::Vector4d line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	Eigen::Vector4d line_p2 = line_pt + line_dir;

	// Iterate through the 3d points and calculate the distances from them to the line
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the line
		// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
		Eigen::Vector4d pt ((*inputPointCloud->getPointCloud())[inliers[i]].getX(), (*inputPointCloud->getPointCloud())[inliers[i]].getY(),
				(*inputPointCloud->getPointCloud())[inliers[i]].getZ(), 0);
		Eigen::Vector4d pp = line_p2 - pt;

#ifdef EIGEN3
		Eigen::Vector3d c = pp.head<3> ().cross (line_dir.head<3> ());
#else
		Eigen::Vector3d c = pp.start<3> ().cross (line_dir.start<3> ());
#endif
		//distances[i] = sqrt (c.dot (c)) / line_dir.dot (line_dir);
		distances[i] = c.dot (c) / line_dir.dot (line_dir);
	}
}


void ObjectModelLine::projectPoints (const std::vector<int> &inliers, const Eigen::VectorXd &model_coefficients,
		PointCloud3D* projectedPointCloud){


	assert (model_coefficients.size () == 6);

	// Obtain the line point and direction
	Eigen::Vector4d line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

	// Iterate through the 3d points and calculate the distances from them to the line
	for (size_t i = 0; i < inliers.size (); ++i)
	{
		Eigen::Vector4d pt ((*inputPointCloud->getPointCloud())[inliers[i]].getX(), (*inputPointCloud->getPointCloud())[inliers[i]].getY(),
				(*inputPointCloud->getPointCloud())[inliers[i]].getZ(), 0);
		// double k = (DOT_PROD_3D (points[i], p21) - dotA_B) / dotB_B;
		double k = (pt.dot (line_dir) - line_pt.dot (line_dir)) / line_dir.dot (line_dir);

		Eigen::Vector4d pp = line_pt + k * line_dir;
		// Calculate the projection of the point on the line (pointProj = A + k * B)
//		std::vector<Point3D> *projectedPoints = projectedPointCloud->getPointCloud();
		(*projectedPointCloud->getPointCloud())[i].setX(pp[0]);
		(*projectedPointCloud->getPointCloud())[i].setY(pp[1]);
		(*projectedPointCloud->getPointCloud())[i].setZ(pp[2]);
	}

}


bool ObjectModelLine::doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXd &model_coefficients,
		double threshold){

	assert (model_coefficients.size () == 6);

     // Obtain the line point and direction
     Eigen::Vector4d line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
     Eigen::Vector4d line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
     Eigen::Vector4d line_p2 = line_pt + line_dir;

     double sqr_threshold = threshold * threshold;
     // Iterate through the 3d points and calculate the distances from them to the line
     for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
     {
       // Calculate the distance from the point to the line
       // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
       Eigen::Vector4d pt ((*inputPointCloud->getPointCloud())[*it].getX(), (*inputPointCloud->getPointCloud())[*it].getY(),
    		   (*inputPointCloud->getPointCloud())[*it].getZ(), 0);
       Eigen::Vector4d pp = line_p2 - pt;

#ifdef EIGEN3
		Eigen::Vector3d c = pp.head<3> ().cross (line_dir.head<3> ());
#else
		Eigen::Vector3d c = pp.start<3> ().cross (line_dir.start<3> ());
#endif
       double sqr_distance = c.dot (c) / line_dir.dot (line_dir);

       if (sqr_distance > sqr_threshold)
         return (false);
     }

     return (true);
}

}
