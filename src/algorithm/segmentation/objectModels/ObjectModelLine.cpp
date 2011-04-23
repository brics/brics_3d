/*
 * ObjectModelLine.cpp
 *
 *  Created on: Apr 23, 2011
 *      Author: reon
 */

#include "ObjectModelLine.h"

namespace BRICS_3D {

void ObjectModelLine::getSamples (int &iterations, std::vector<int> &samples){
	// We're assuming that indices_ have already been set in the constructor
	//ToDo ROS_ASSERT (this->indices_->size () != 0);

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
		Eigen::VectorXf &model_coefficients){
	// Need 2 samples
	//ToDo ROS_ASSERT (samples.size () == 2);

	model_coefficients.resize (6);
	model_coefficients[0] = this->points->data()[samples[0]].getX();
	model_coefficients[1] = this->points->data()[samples[0]].getY();
	model_coefficients[2] = this->points->data()[samples[0]].getZ();

	model_coefficients[3] = this->points->data()[samples[1]].getX() - model_coefficients[0];
	model_coefficients[4] = this->points->data()[samples[1]].getY() - model_coefficients[1];
	model_coefficients[5] = this->points->data()[samples[1]].getZ() - model_coefficients[2];

	model_coefficients.end<3> ().normalize ();
	return (true);
}

void ObjectModelLine::optimizeModelCoefficients (const std::vector<int> &inliers,
		const Eigen::VectorXf &model_coefficients,
		Eigen::VectorXf &optimized_coefficients){
	// Needs a valid set of model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 6);

	if (inliers.size () == 0)
	{
		cout<<"[ObjectModelLine::optimizeModelCoefficients] Inliers vector empty! Returning the same coefficients";
		optimized_coefficients = model_coefficients;
		return;
	}

	// Need at least 2 points to estimate a line
	//ToDo ROS_ASSERT (inliers.size () > 2);

	optimized_coefficients.resize (6);

	// Compute the 3x3 covariance matrix
	Eigen::Vector4f centroid;
	compute3DCentroid (this->inputPointCloud, inliers, centroid);
	Eigen::Matrix3f covariance_matrix;
	computeCovarianceMatrix (inputPointCloud, inliers, centroid, covariance_matrix);
	optimized_coefficients[0] = centroid[0];
	optimized_coefficients[1] = centroid[1];
	optimized_coefficients[2] = centroid[2];

	// Extract the eigenvalues and eigenvectors
	//cloud_geometry::eigen_cov (covariance_matrix, eigen_values, eigen_vectors);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> ei_symm (covariance_matrix);
	EIGEN_ALIGN_128 Eigen::Vector3f eigen_values  = ei_symm.eigenvalues ();
	EIGEN_ALIGN_128 Eigen::Matrix3f eigen_vectors = ei_symm.eigenvectors ();

	optimized_coefficients.end<3> () = eigen_vectors.col (2).normalized ();
}


void ObjectModelLine::getDistancesToModel (const Eigen::VectorXf &model_coefficients,
		std::vector<double> &distances){
	// Needs a valid set of model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 6);

	distances.resize (this->inputPointCloud->getSize());

	// Obtain the line point and direction
	Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	Eigen::Vector4f line_p2 = line_pt + line_dir;

	// Iterate through the 3d points and calculate the distances from them to the line
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the line
		// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
		Eigen::Vector4f pt (this->points->data()[i].getX(), this->points->data()[i].getY(),
				this->points->data()[i].getZ(), 0);
		Eigen::Vector4f pp = line_p2 - pt;

		Eigen::Vector3f c = pp.start<3> ().cross (line_dir.start<3> ());
		//distances[i] = sqrt (c.dot (c)) / line_dir.dot (line_dir);
		distances[i] = c.dot (c) / line_dir.dot (line_dir);
	}
}

void ObjectModelLine::selectWithinDistance (const Eigen::VectorXf &model_coefficients, double threshold,
		std::vector<int> &inliers){
	// Needs a valid set of model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 6);

	double sqr_threshold = threshold * threshold;

	int nr_p = 0;
	inliers.resize (this->inputPointCloud->getSize());

	// Obtain the line point and direction
	Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	Eigen::Vector4f line_p2 = line_pt + line_dir;

	// Iterate through the 3d points and calculate the distances from them to the line
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the line
		// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
		Eigen::Vector4f pt (this->points->data()[i].getX(),
				this->points->data()[i].getY(), this->points->data()[i].getZ(), 0);
		Eigen::Vector4f pp = line_p2 - pt;

		Eigen::Vector3f c = pp.start<3> ().cross (line_dir.start<3> ());
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


void ObjectModelLine::getInlierDistance (std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
		std::vector<double> &distances){
	// Needs a valid set of model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 6);

	distances.resize (this->inputPointCloud->getSize());

	// Obtain the line point and direction
	Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
	Eigen::Vector4f line_p2 = line_pt + line_dir;

	// Iterate through the 3d points and calculate the distances from them to the line
	for (size_t i = 0; i < this->inputPointCloud->getSize(); ++i)
	{
		// Calculate the distance from the point to the line
		// D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
		Eigen::Vector4f pt (this->points->data()[inliers[i]].getX(), this->points->data()[inliers[i]].getY(),
				this->points->data()[inliers[i]].getZ(), 0);
		Eigen::Vector4f pp = line_p2 - pt;

		Eigen::Vector3f c = pp.start<3> ().cross (line_dir.start<3> ());
		//distances[i] = sqrt (c.dot (c)) / line_dir.dot (line_dir);
		distances[i] = c.dot (c) / line_dir.dot (line_dir);
	}
}


void ObjectModelLine::projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
		PointCloud3D* projectedPointCloud){

	// Needs a valid model coefficients
	//ToDo ROS_ASSERT (model_coefficients.size () == 6);

	// Obtain the line point and direction
	Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
	Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

	// Iterate through the 3d points and calculate the distances from them to the line
	for (size_t i = 0; i < inliers.size (); ++i)
	{
		Eigen::Vector4f pt (this->points->data()[inliers[i]].getX(), this->points->data()[inliers[i]].getY(),
				this->points->data()[inliers[i]].getZ(), 0);
		// double k = (DOT_PROD_3D (points[i], p21) - dotA_B) / dotB_B;
		double k = (pt.dot (line_dir) - line_pt.dot (line_dir)) / line_dir.dot (line_dir);

		Eigen::Vector4f pp = line_pt + k * line_dir;
		// Calculate the projection of the point on the line (pointProj = A + k * B)
		std::vector<Point3D> *projectedPoints = projectedPointCloud->getPointCloud();
		projectedPoints->data()[i].setX(pp[0]);
		projectedPoints->data()[i].setY(pp[1]);
		projectedPoints->data()[i].setZ(pp[2]);
	}

}


bool ObjectModelLine::doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients,
		double threshold){
    // Needs a valid set of model coefficients
     //ToDo ROS_ASSERT (model_coefficients.size () == 6);

     // Obtain the line point and direction
     Eigen::Vector4f line_pt  (model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
     Eigen::Vector4f line_dir (model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
     Eigen::Vector4f line_p2 = line_pt + line_dir;

     double sqr_threshold = threshold * threshold;
     // Iterate through the 3d points and calculate the distances from them to the line
     for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
     {
       // Calculate the distance from the point to the line
       // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
       Eigen::Vector4f pt (this->points->data()[*it].getX(), this->points->data()[*it].getY(),
    		   this->points->data()[*it].getZ(), 0);
       Eigen::Vector4f pp = line_p2 - pt;

       Eigen::Vector3f c = pp.start<3> ().cross (line_dir.start<3> ());
       double sqr_distance = c.dot (c) / line_dir.dot (line_dir);

       if (sqr_distance > sqr_threshold)
         return (false);
     }

     return (true);
}

}
