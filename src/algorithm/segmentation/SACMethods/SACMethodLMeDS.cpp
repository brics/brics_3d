/*
 * @file: SACMethodLMeDS.cpp
 *
 * @date: Apr 21, 2011
 * @author: reon
 */

#include "SACMethodLMeDS.h"
#include <algorithm>

namespace brics_3d {

SACMethodLMeDS::SACMethodLMeDS() {}

SACMethodLMeDS::~SACMethodLMeDS() {}

bool SACMethodLMeDS::computeModel(){
	// Warn and exit if no threshold was set
	if (this->threshold == -1)
	{
		cout<<"[LMeDS::computeModel] No threshold set!"<<endl;
		return (false);
	}

	this->iterations = 0;
	double d_best_penalty = DBL_MAX;

	std::vector<int> bestModelInliersFound;
	std::vector<int> selection;
	Eigen::VectorXd estimatedModelCoefficients;
	std::vector<double> distances;

	int noInliersCurrentModel = 0;

	// Iterate
	while (this->iterations < this->maxIterations)
	{
	       //Compute a random model from the input pointcloud
	       bool isDegenerate = false;
	       bool modelFound = false;

	       this->objectModel->computeRandomModel(this->iterations,estimatedModelCoefficients,isDegenerate,modelFound);

	       if (!isDegenerate) break;

	       if(!modelFound){
	    	   this->iterations++;
	    	   continue;
	       }

		double currentModelDistancePenalty = 0;


		// Iterate through the 3d points and calculate the distances from them to the model
		this->objectModel->getDistancesToModel (estimatedModelCoefficients, distances);
		std::sort (distances.begin (), distances.end ());


		int mid = this->objectModel->getInputCloud()->getSize() / 2;

		if (this->objectModel->getInputCloud()->getSize() % 2 == 0)
			// d_cur_penalty = median (distances)
			currentModelDistancePenalty = (sqrt (distances[mid-1]) + sqrt (distances[mid])) / 2;
		else
			currentModelDistancePenalty = sqrt (distances[mid]);

		if (currentModelDistancePenalty < d_best_penalty)
		{
			d_best_penalty = currentModelDistancePenalty;

			// Save the current model-coefficients selection as being the best so far
			this->modelCoefficients = estimatedModelCoefficients;
		}

		this->iterations++;
	}

	// Iterate through the 3d points and calculate the distances from them to the model again
	this->objectModel->getDistancesToModel (this->modelCoefficients, distances);

	this->inliers.resize (distances.size ());
	// Get the inliers for the best model found
	noInliersCurrentModel = 0;
	for (size_t i = 0; i < distances.size (); ++i)
		if (distances[i] <= this->threshold)
			this->inliers[noInliersCurrentModel++] = i;

	// Resize the inliers vector
	this->inliers.resize (noInliersCurrentModel);

	if (this->inliers.size () == 0)
		{
			cout<<"[LMeDS::computeModel] Unable to find a solution!"<<endl;
			return (false);
		}
	return (true);
}
}

