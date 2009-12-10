/**
 * @file 
 * IterativeClosestPoint.cpp
 *
 * @date: Dec 3, 2009
 * @author: sblume
 */

#include "IterativeClosestPoint.h"
#include "core/HomogeneousMatrix44.h" //TODO? now it depends  on implementation of HomogeneousMatrix44
#include <cmath>
#include <assert.h>

using std::cout;
using std::endl;

namespace BRICS_3D {

IterativeClosestPoint::IterativeClosestPoint() {
	assigner = 0;
	estimator = 0;
	convergenceThreshold = 0.00001;
	maxIterations = 20;
}

IterativeClosestPoint::IterativeClosestPoint(IPointCorrespondence *assigner, IRigidTransformationEstimation *estimator, double convergenceThreshold, int maxIterations) {
	this->assigner = assigner;
	this->estimator = estimator;
	this->convergenceThreshold = convergenceThreshold;
	this->maxIterations = maxIterations;
}

IterativeClosestPoint::~IterativeClosestPoint() {
	delete assigner;
	delete estimator;
}

void IterativeClosestPoint::match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation) {
	assert(assigner != 0); //check if algorithms are setup
	assert(estimator != 0);

	double error = 0.0;
	double previousError = 0.0;
	double previousPreviousError = 0.0;

	IHomogeneousMatrix44* tmpResultTransformation = new HomogeneousMatrix44();

	/* perform generic ICP */
	for (int i = 0; i < maxIterations; ++i) {
		previousPreviousError = previousError;
		previousError = error;

		/* find closest points */
		std::vector<CorrespondencePoint3DPair>* pointPairs = new std::vector<CorrespondencePoint3DPair>();
		assigner->createNearestNeighborCorrespondence(model, data, pointPairs);

		/* estimate transformation */
		error = estimator->estimateTransformation(pointPairs, tmpResultTransformation);
//		cout << "Estimated transformation: " << endl  << *tmpResultTransformation; //DBG output
		*resultTransformation = *((*resultTransformation) * (*tmpResultTransformation)); // accumulate transformations

		/* perform transformation on data point cloud */
		data->homogeneousTransformation(tmpResultTransformation);

		/* stop if error is below convergence threshold */
		if ((std::abs(error - previousError) < convergenceThreshold) &&
				(std::abs(error - previousPreviousError) < convergenceThreshold)) {
			cout << "INFO: ICP converged after " << i << " iterations. " << endl; //DBG output
			break;
		}
	}

	cout << "INFO: RMS Error is: " << error << endl; //DBG output
	delete tmpResultTransformation;
}

void IterativeClosestPoint::match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation, IHomogeneousMatrix44* initalEstimate, int maxIterations) {

}

IPointCorrespondence* IterativeClosestPoint::getAssigner() const
{
    return assigner;
}

double IterativeClosestPoint::getConvergenceThreshold() const{
	return convergenceThreshold;
}

IRigidTransformationEstimation* IterativeClosestPoint::getEstimator() const {
	return estimator;
}

int IterativeClosestPoint::getMaxIterations() const {
	return maxIterations;
}

void IterativeClosestPoint::setAssigner(IPointCorrespondence* assigner)
{
	this->assigner = assigner;
}

void IterativeClosestPoint::setConvergenceThreshold(double convergenceThreshold)
{
	this->convergenceThreshold = convergenceThreshold;
}

void IterativeClosestPoint::setEstimator(IRigidTransformationEstimation* estimator)
{
	this->estimator = estimator;
}

void IterativeClosestPoint::setMaxIterations(int maxIterations)
{
	this->maxIterations = maxIterations;
}


}

/* EOF */
