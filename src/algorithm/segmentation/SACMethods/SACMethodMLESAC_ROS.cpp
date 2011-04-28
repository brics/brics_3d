/*
 * SACMethodMLESAC_ROS.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: reon
 */

#include "SACMethodMLESAC_ROS.h"

namespace BRICS_3D {

SACMethodMLESAC_ROS::SACMethodMLESAC_ROS() {
	this->iterationsEM = 3;
}

SACMethodMLESAC_ROS::~SACMethodMLESAC_ROS() {}

bool SACMethodMLESAC_ROS::computeModel(){
	// Warn and exit if no threshold was set
	if (this->threshold == -1)
	{
		cout<<"[MLESAC::computeModel] No threshold set!"<<endl;
		return (false);
	}

	this->iterations = 0;
	double d_best_penalty = DBL_MAX;
	double k = 1.0;

	std::vector<int> best_model;
	Eigen::VectorXd model_coefficients;
	std::vector<double> distances;

	// Compute sigma - remember to set threshold_ correctly !
	sigma_ = computeMedianAbsoluteDeviation (this->objectModel->getInputCloud (), this->threshold);
	cout<<"[MLESAC::computeModel] Estimated sigma value: "<< sigma_<<endl;

	// Compute the bounding box diagonal: V = sqrt (sum (max(pointCloud) - min(pointCloud)^2))
	Eigen::Vector4d min_pt, max_pt;
	getMinMax (this->objectModel->getInputCloud (), min_pt, max_pt);
	max_pt -= min_pt;
	double v = sqrt (max_pt.dot (max_pt));

	int n_inliers_count = 0;
	size_t indices_size;
	// Iterate
	while (this->iterations < k)
	{
	       //Compute a random model from the input pointcloud
	       bool isDegenerate = false;
	       bool modelFound = false;

	       this->objectModel->computeRandomModel(this->iterations,model_coefficients,isDegenerate,modelFound);

	       if (!isDegenerate) break;

	       if(!modelFound){
	    	   this->iterations++;
	    	   continue;
	       }

		// Iterate through the 3d points and calculate the distances from them to the model
		this->objectModel->getDistancesToModel (model_coefficients, distances);

		// Use Expectiation-Maximization to find out the right value for d_cur_penalty
		// ---[ Initial estimate for the gamma mixing parameter = 1/2
		double gamma = 0.5;
		double p_outlier_prob = 0;

		indices_size = this->objectModel->getInputCloud()->getSize();
		std::vector<double> p_inlier_prob (indices_size);
		for (int j = 0; j < iterationsEM; ++j)
		{
			// Likelihood of a datum given that it is an inlier
			for (size_t i = 0; i < indices_size; ++i)
				p_inlier_prob[i] = gamma * exp (- (distances[i] * distances[i] ) / 2 * (sigma_ * sigma_) ) /
				(sqrt (2 * M_PI) * sigma_);

			// Likelihood of a datum given that it is an outlier
			p_outlier_prob = (1 - gamma) / v;

			gamma = 0;
			for (size_t i = 0; i < indices_size; ++i)
				gamma += p_inlier_prob [i] / (p_inlier_prob[i] + p_outlier_prob);
			gamma /= this->objectModel->getInputCloud()->getSize();
		}

		// Find the log likelihood of the model -L = -sum [log (pInlierProb + pOutlierProb)]
		double d_cur_penalty = 0;
		for (size_t i = 0; i < indices_size; ++i)
			d_cur_penalty += log (p_inlier_prob[i] + p_outlier_prob);
		d_cur_penalty = - d_cur_penalty;

		// Better match ?
		if (d_cur_penalty < d_best_penalty)
		{
			d_best_penalty = d_cur_penalty;

			// Save the current model/coefficients selection as being the best so far
			this->modelCoefficients = model_coefficients;

			n_inliers_count = 0;
			// Need to compute the number of inliers for this model to adapt k
			for (size_t i = 0; i < distances.size (); ++i)
				if (distances[i] <= 2 * sigma_)
					n_inliers_count++;

			// Compute the k parameter (k=log(z)/log(1-w^n))
			double w = (double)((double)n_inliers_count / (double)this->objectModel->getInputCloud()->getSize());
			double p_no_outliers = 1 - pow (w, (double)this->objectModel->getNumberOfSamplesRequired());
			p_no_outliers = std::max (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
			p_no_outliers = std::min (1 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
			k = log (1 - this->probability) / log (p_no_outliers);
		}

		this->iterations++;

		if (this->iterations > this->maxIterations)
		{
			cout<<"[MLESAC::computeModel] MLESAC reached the maximum number of trials"<<endl;
			break;
		}
	}

	// Iterate through the 3d points and calculate the distances from them to the model again
	this->objectModel->getDistancesToModel (this->modelCoefficients, distances);

	this->inliers.resize (distances.size ());
	// Get the inliers for the best model found
	n_inliers_count = 0;
	for (size_t i = 0; i < distances.size (); ++i)
		if (distances[i] <= 2 * sigma_)
			this->inliers[n_inliers_count++] = i;

	// Resize the inliers vector
	this->inliers.resize (n_inliers_count);

	if (this->inliers.size () == 0)
	{
		cout<<"[MLESAC::computeModel] Unable to find a solution!"<<endl;
		return (false);
	}
	return (true);

}

void SACMethodMLESAC_ROS::computeMedian (PointCloud3D *cloud, Eigen::Vector4f &median){
	// Copy the values to vectors for faster sorting
	std::vector<Coordinate> x(cloud->getSize());
	std::vector<Coordinate> y(cloud->getSize());
	std::vector<Coordinate> z(cloud->getSize());

	std::vector<Point3D> *points;
	points= cloud->getPointCloud();

	for (size_t i = 0; i < cloud->getSize(); ++i)
	{
		x[i] = points->data()[i].getX();
		y[i] = points->data()[i].getY();;
		z[i] = points->data()[i].getZ();;
	}

	std::sort (x.begin (), x.end ());
	std::sort (y.begin (), y.end ());
	std::sort (z.begin (), z.end ());

	int mid = cloud->getSize () / 2;
	if (cloud->getSize () % 2 == 0)
	{
		median[0] = (x[mid-1] + x[mid]) / 2;
		median[1] = (y[mid-1] + y[mid]) / 2;
		median[2] = (z[mid-1] + z[mid]) / 2;
	}
	else
	{
		median[0] = x[mid];
		median[1] = y[mid];
		median[2] = z[mid];
	}
	median[3] = 0;
}

double SACMethodMLESAC_ROS::computeMedianAbsoluteDeviation (PointCloud3D *cloud,double sigma) {

	std::vector<double> distances (cloud->getSize());

	Eigen::Vector4f median;
	// median (dist (x - median (x)))
	computeMedian (cloud, median);

	std::vector<Point3D> *points = cloud->getPointCloud();

	Eigen::Vector4f pt, ptdiff;
	{
		for (size_t i = 0; i < cloud->getSize(); ++i)
		{
			pt = Eigen::Vector4f (points->data()[i].getX(), points->data()[i].getY(), points->data()[i].getZ(), 0);
			pt -= median;
			distances[i] = pt.dot (pt);
		}
	}

	std::sort (distances.begin (), distances.end ());

	double result;
	int mid = cloud->getSize() / 2;
	// Do we have a "middle" point or should we "estimate" one ?
	if (cloud->getSize() % 2 == 0)
		result = (sqrt (distances[mid-1]) + sqrt (distances[mid])) / 2;
	else
		result = sqrt (distances[mid]);
	return (sigma * result);
}

void SACMethodMLESAC_ROS::getMinMax (PointCloud3D *cloud, Eigen::Vector4d &min_p, Eigen::Vector4d &max_p){
	/** \brief Determine the minimum and maximum 3D bounding box coordinates for a given set of points
	 * \param cloud the point cloud message
	 * \param indices the set of point indices to use
	 * \param min_p the resultant minimum bounding box coordinates
	 * \param max_p the resultant maximum bounding box coordinates
	 */
	min_p.setConstant (DBL_MAX);
	max_p.setConstant (-DBL_MAX);
	min_p[3] = max_p[3] = 0;

	std::vector<Point3D> *points = cloud->getPointCloud();

	for (size_t i = 0; i < cloud->getSize(); ++i)
	{
		if (points->data()[i].getX() < min_p[0]) min_p[0] = points->data()[i].getX();
		if (points->data()[i].getY() < min_p[1]) min_p[1] = points->data()[i].getY();
		if (points->data()[i].getZ() < min_p[2]) min_p[2] = points->data()[i].getZ();

		if (points->data()[i].getX() > max_p[0]) max_p[0] = points->data()[i].getX();
		if (points->data()[i].getY() > max_p[1]) max_p[1] = points->data()[i].getY();
		if (points->data()[i].getZ() > max_p[2]) max_p[2] = points->data()[i].getZ();
	}

}


}
