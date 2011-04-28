/*
 * SACMethodLMeDS_ROS.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: reon
 */

#include "SACMethodLMeDS_ROS.h"
#include <algorithm>

namespace BRICS_3D {

SACMethodLMeDS_ROS::SACMethodLMeDS_ROS() {
	// TODO Auto-generated constructor stub

}

SACMethodLMeDS_ROS::~SACMethodLMeDS_ROS() {
	// TODO Auto-generated destructor stub
}

bool SACMethodLMeDS_ROS::computeModel(){
	// Warn and exit if no threshold was set
	if (this->threshold == -1)
	{
		cout<<"[LMeDS::computeModel] No threshold set!"<<endl;
		return (false);
	}

	this->iterations = 0;
	double d_best_penalty = DBL_MAX;

	std::vector<int> best_model;
	std::vector<int> selection;
	Eigen::VectorXd model_coefficients;
	std::vector<double> distances;

	int n_inliers_count = 0;

	// Iterate
	while (this->iterations < this->maxIterations)
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

		double d_cur_penalty = 0;
		// d_cur_penalty = sum (min (dist, threshold))

		// Iterate through the 3d points and calculate the distances from them to the model
		this->objectModel->getDistancesToModel (model_coefficients, distances);
		std::sort (distances.begin (), distances.end ());
		// d_cur_penalty = median (distances)
		int mid = this->objectModel->getInputCloud()->getSize() / 2;

		// Do we have a "middle" point or should we "estimate" one ?
		if (this->objectModel->getInputCloud()->getSize() % 2 == 0)
			d_cur_penalty = (sqrt (distances[mid-1]) + sqrt (distances[mid])) / 2;
		else
			d_cur_penalty = sqrt (distances[mid]);

		// Better match ?
		if (d_cur_penalty < d_best_penalty)
		{
			d_best_penalty = d_cur_penalty;

			// Save the current model/coefficients selection as being the best so far
			this->modelCoefficients = model_coefficients;
		}

		this->iterations++;
	}

	// Classify the data points into inliers and outliers
	// Sigma = 1.4826 * (1 + 5 / (n-d)) * sqrt (M)
	// @note: See "Robust Regression Methods for Computer Vision: A Review"
	//double sigma = 1.4826 * (1 + 5 / (sac_model_->getIndices ()->size () - best_model.size ())) * sqrt (d_best_penalty);
	//double threshold = 2.5 * sigma;

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

	if (this->inliers.size () == 0)
		{
			cout<<"[LMeDS::computeModel] Unable to find a solution!"<<endl;
			return (false);
		}
	return (true);
}
}

