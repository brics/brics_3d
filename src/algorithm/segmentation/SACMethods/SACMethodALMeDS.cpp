/*
 * @file: SACMethodALMeDS.cpp
 *
 * @date: Apr 21, 2011
 * @author: reon
 */


#include "SACMethodALMeDS.h"
#include <algorithm>
namespace BRICS_3D {

SACMethodALMeDS::SACMethodALMeDS() {}

SACMethodALMeDS::~SACMethodALMeDS() {}

bool SACMethodALMeDS::computeModel(){



	// Warn and exit if no threshold was set
	if (this->threshold == -1)
	{
		cout<<"[ALMeDS::computeModel] No threshold set!"<<endl;
		return (false);
	}

	this->iterations = 0;
	double bestPenaltyFound = DBL_MAX;

	std::vector<int> bestModelInliers;
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


		double crrentModelPenalty = 0;
		std::vector<int> inliers;

		//Find the points inside threshold distance to the model
		this->objectModel->selectWithinDistance (estimatedModelCoefficients, this->threshold, inliers);

		distances.resize (inliers.size ());
		// Iterate through the inliers and calculate the distances from them to the model
		this->objectModel->getInlierDistance (inliers, estimatedModelCoefficients, distances);
		std::sort (distances.begin (), distances.end ());
		int mid = inliers.size () / 2;

		if (inliers.size () % 2 == 0)
			// d_cur_penalty = median (distances)
			crrentModelPenalty = (sqrt (distances[mid-1]) + sqrt (distances[mid])) / 2;
		else
			crrentModelPenalty = sqrt (distances[mid]);

		if (crrentModelPenalty < bestPenaltyFound)
		{
			bestPenaltyFound = crrentModelPenalty;

			// Save the current model/coefficients selection as being the best so far
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
				cout<<"[ALMeDS::computeModel] Unable to find a solution!"<<endl;
				return (false);
			}
	return (true);
}

}
