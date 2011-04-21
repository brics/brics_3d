/*
 * SACMethodMSAC_ROS.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: reon
 */

#include "SACMethodMSAC_ROS.h"

namespace BRICS_3D {

SACMethodMSAC_ROS::SACMethodMSAC_ROS() {
	// TODO Auto-generated constructor stub

}

SACMethodMSAC_ROS::~SACMethodMSAC_ROS() {
	// TODO Auto-generated destructor stub
}

bool SACMethodMSAC_ROS::computeModel(){

	// Warn and exit if no threshold was set
	if (this->threshold == -1)
	{
		cout<<"[MSAC::computeModel] No threshold set!"<<endl;
		return (false);
	}

	this->iterations = 0;
	double d_best_penalty = 10000;
	double k = 1.0;

	std::vector<int> best_model;
	std::vector<int> selection;
	Eigen::VectorXf model_coefficients;
	std::vector<double> distances;

	int n_inliers_count = 0;
	// Iterate
	while (this->iterations < k)
	{
		// Get X samples which satisfy the model criteria
		this->objectModel->getSamples (this->iterations, selection);

		if (selection.size () == 0) break;

		// Search for inliers in the point cloud for the current plane model M
		if (!this->objectModel->computeModelCoefficients (selection, model_coefficients))
		{
			this->iterations++;
			continue;
		}

		double d_cur_penalty = 0;
		// Iterate through the 3d points and calculate the distances from them to the model
		this->objectModel->getDistancesToModel (model_coefficients, distances);
		for (size_t i = 0; i < distances.size (); ++i)
			d_cur_penalty += std::min (distances[i], this->threshold);

		// Better match ?
		if (d_cur_penalty < d_best_penalty)
		{
			d_best_penalty = d_cur_penalty;

			// Save the current model/coefficients selection as being the best so far
			this->model              = selection;
			this->modelCoefficients = model_coefficients;

			n_inliers_count = 0;
			// Need to compute the number of inliers for this model to adapt k
			for (size_t i = 0; i < distances.size (); ++i)
				if (distances[i] <= this->threshold)
					n_inliers_count++;

			// Compute the k parameter (k=log(z)/log(1-w^n))
			double w = (double)((double)n_inliers_count / (double)this->objectModel->getInputCloud()->getSize());
			double p_no_outliers = 1 - pow (w, (double)selection.size ());
			p_no_outliers = std::max (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
			p_no_outliers = std::min (1 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
			k = log (1 - this->probability) / log (p_no_outliers);
		}

		this->iterations++;

		if (this->iterations > this->maxIterations)
		{

				cout<<"[MSAC::computeModel] MSAC reached the maximum number of trials."<<endl;
			break;
		}
	}

	if (this->model.size () == 0)
	{
		cout<<"[MSAC::computeModel] Unable to find a solution!"<<endl;
		return (false);
	}

	// Iterate through the 3d points and calculate the distances from them to the model again
	this->objectModel->getDistancesToModel (this->modelCoefficients, distances);

	this->inliers.resize (distances.size ());
	// Get the inliers for the best model found
	n_inliers_count = 0;
	for (size_t i = 0; i < distances.size (); ++i)
		if (distances[i] <= this->threshold)
			this->inliers[n_inliers_count++] = i;

	// Resize the inliers vector
	this->inliers.resize (n_inliers_count);

	return (true);

}

}
