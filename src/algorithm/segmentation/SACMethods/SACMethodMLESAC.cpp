/*
 * @file: SACMethodMLESAC.cpp
 *
 * @date: Created on: Apr 21, 2011
 * @author: reon
 */

#include "SACMethodMLESAC.h"

namespace BRICS_3D {

SACMethodMLESAC::SACMethodMLESAC() {
	//Default value used
	this->iterationsEM = 3;
}

SACMethodMLESAC::~SACMethodMLESAC() {}

bool SACMethodMLESAC::computeModel(){
	// Warn and exit if no threshold was set
	if (this->threshold == -1)
	{
		cout<<"[MLESAC::computeModel] No threshold set!"<<endl;
		return (false);
	}

	this->iterations = 0;
	double bestDistancePenaltyFound = DBL_MAX;
	double k = 1.0;

	std::vector<int> bestModelIndices;
	Eigen::VectorXd estimatedModelCoefficients;
	std::vector<double> distances;

	// Compute sigma - remember to set threshold_ correctly !
	sigma_ = computeMedianAbsoluteDeviation (this->objectModel->getInputCloud (), this->threshold);
	cout<<"[MLESAC::computeModel] Estimated sigma value: "<< sigma_<<endl;

	// Compute the bounding box diagonal: V = sqrt (sum (max(pointCloud) - min(pointCloud)^2))
	Eigen::Vector4d minPt, maxPt;
	getMinMax (this->objectModel->getInputCloud (), minPt, maxPt);
	maxPt -= minPt;
	double v = sqrt (maxPt.dot (maxPt));

	int noInliersCurrentModel = 0;
	size_t indicesSize;
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

		// Iterate through the 3d points and calculate the distances from them to the model
		this->objectModel->getDistancesToModel (estimatedModelCoefficients, distances);

		// Use Expectiation-Maximization to find out the right value for d_cur_penalty
		// ---[ Initial estimate for the gamma mixing parameter = 1/2
		double gamma = 0.5;
		double outlierProbability = 0;

		indicesSize = this->objectModel->getInputCloud()->getSize();
		std::vector<double> inlierProbability (indicesSize);
		for (int j = 0; j < iterationsEM; ++j)
		{
			// Likelihood of a datum given that it is an inlier
			for (size_t i = 0; i < indicesSize; ++i)
				inlierProbability[i] = gamma * exp (- (distances[i] * distances[i] ) / 2 * (sigma_ * sigma_) ) /
				(sqrt (2 * M_PI) * sigma_);

			// Likelihood of a datum given that it is an outlier
			outlierProbability = (1 - gamma) / v;

			gamma = 0;
			for (size_t i = 0; i < indicesSize; ++i)
				gamma += inlierProbability [i] / (inlierProbability[i] + outlierProbability);
			gamma /= this->objectModel->getInputCloud()->getSize();
		}

		// Find the log likelihood of the model -L = -sum [log (pInlierProb + pOutlierProb)]
		double currentPenalty = 0;
		for (size_t i = 0; i < indicesSize; ++i)
			currentPenalty += log (inlierProbability[i] + outlierProbability);
		currentPenalty = - currentPenalty;

		// Better match ?
		if (currentPenalty < bestDistancePenaltyFound)
		{
			bestDistancePenaltyFound = currentPenalty;

			// Save the current model/coefficients selection as being the best so far
			this->modelCoefficients = estimatedModelCoefficients;

			noInliersCurrentModel = 0;
			// Need to compute the number of inliers for this model to adapt k
			for (size_t i = 0; i < distances.size (); ++i)
				if (distances[i] <= 2 * sigma_)
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
			cout<<"[MLESAC::computeModel] MLESAC reached the maximum number of trials"<<endl;
			break;
		}
	}

	// Iterate through the 3d points and calculate the distances from them to the model again
	this->objectModel->getDistancesToModel (this->modelCoefficients, distances);

	this->inliers.resize (distances.size ());
	// Get the inliers for the best model found
	noInliersCurrentModel = 0;
	for (size_t i = 0; i < distances.size (); ++i)
		if (distances[i] <= 2 * sigma_)
			this->inliers[noInliersCurrentModel++] = i;

	// Resize the inliers vector
	this->inliers.resize (noInliersCurrentModel);

	if (this->inliers.size () == 0)
	{
		cout<<"[MLESAC::computeModel] Unable to find a solution!"<<endl;
		return (false);
	}
	return (true);

}

void SACMethodMLESAC::computeMedian (PointCloud3D *cloud, Eigen::Vector4f &median){
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

double SACMethodMLESAC::computeMedianAbsoluteDeviation (PointCloud3D *cloud,double sigma) {

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

void SACMethodMLESAC::getMinMax (PointCloud3D *cloud, Eigen::Vector4d &minP, Eigen::Vector4d &maxP){
	minP.setConstant (DBL_MAX);
	maxP.setConstant (-DBL_MAX);
	minP[3] = maxP[3] = 0;

	std::vector<Point3D> *points = cloud->getPointCloud();

	for (size_t i = 0; i < cloud->getSize(); ++i)
	{
		if (points->data()[i].getX() < minP[0]) minP[0] = points->data()[i].getX();
		if (points->data()[i].getY() < minP[1]) minP[1] = points->data()[i].getY();
		if (points->data()[i].getZ() < minP[2]) minP[2] = points->data()[i].getZ();

		if (points->data()[i].getX() > maxP[0]) maxP[0] = points->data()[i].getX();
		if (points->data()[i].getY() > maxP[1]) maxP[1] = points->data()[i].getY();
		if (points->data()[i].getZ() > maxP[2]) maxP[2] = points->data()[i].getZ();
	}

}


}
