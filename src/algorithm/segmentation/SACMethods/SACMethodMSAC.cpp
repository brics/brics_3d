/*
 * @file: SACMethodMSAC.cpp
 *
 * @date: Apr 21, 2011
 * @author: reon
 */

#include "SACMethodMSAC.h"

namespace brics_3d {

SACMethodMSAC::SACMethodMSAC() {}

SACMethodMSAC::~SACMethodMSAC() {}

bool SACMethodMSAC::computeModel(){

	// Warn and exit if no threshold was set
	if (this->threshold == -1)
	{
		cout<<"[MSAC::computeModel] No threshold set!"<<endl;
		return (false);
	}

	this->iterations = 0;
	double minResidualPenaltyFound = DBL_MAX;
	double k = 1.0;

	std::vector<int> bestModelInliersFound;
	Eigen::VectorXd estimatedModelCoefficients;
	std::vector<double> distances;

	int noInliersCurrentModel = 0;
	// Iterate
	while (this->iterations < k)
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

		double currentResidualPenalty = 0;
		// Iterate through the 3d points and calculate the distances from them to the model
		this->objectModel->getDistancesToModel (estimatedModelCoefficients, distances);
		for (size_t i = 0; i < distances.size (); ++i)
			currentResidualPenalty += std::min (distances[i], this->threshold);

		// Better match ?
		if (currentResidualPenalty < minResidualPenaltyFound)
		{
			minResidualPenaltyFound = currentResidualPenalty;

			// Save the current model/coefficients selection as being the best so far
			this->modelCoefficients = estimatedModelCoefficients;

			noInliersCurrentModel = 0;
			// Need to compute the number of inliers for this model to adapt k
			for (size_t i = 0; i < distances.size (); ++i)
				if (distances[i] <= this->threshold)
					noInliersCurrentModel++;

			// Compute the k parameter (k=log(z)/log(1-w^n))
			double w = (double)((double)noInliersCurrentModel / (double)this->objectModel->getInputCloud()->getSize());
			double pNoOutliers = 1 - pow (w, (double)this->objectModel->getNumberOfSamplesRequired());
			pNoOutliers = std::max (std::numeric_limits<double>::epsilon (), pNoOutliers);       // Avoid division by -Inf
			pNoOutliers = std::min (1 - std::numeric_limits<double>::epsilon (), pNoOutliers);   // Avoid division by 0.
			k = log (1 - this->probability) / log (pNoOutliers);
		}

		this->iterations++;

		if (this->iterations > this->maxIterations)
		{

				cout<<"[MSAC::computeModel] MSAC reached the maximum number of trials."<<endl;
			break;
		}
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
		cout<<"[MSAC::computeModel] Unable to find a solution!"<<endl;
		return (false);
	}

	return (true);

}

}
