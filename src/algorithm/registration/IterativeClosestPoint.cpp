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
	this->assigner = 0;
	this->estimator = 0;
	this->convergenceThreshold = 0.00001;
	this->maxIterations = 20;

	/* initial values fir stateful interface */
	this->model = 0;
	this->data = 0;
	this->intermadiateTransformation = 0;
	this->resultTransformation = 0;
}

IterativeClosestPoint::IterativeClosestPoint(IPointCorrespondence *assigner, IRigidTransformationEstimation *estimator, double convergenceThreshold, int maxIterations) {
	this->assigner = assigner;
	this->estimator = estimator;
	this->convergenceThreshold = convergenceThreshold;
	this->maxIterations = maxIterations;

	/* initial values for stateful interface */
	this->model = 0;
	this->data = 0;
	this->intermadiateTransformation = 0;
	this->resultTransformation = 0;
}

IterativeClosestPoint::~IterativeClosestPoint() {
	delete this->assigner;
	delete this->estimator;

	if (this->intermadiateTransformation != 0) { // might be unused
		delete this->intermadiateTransformation;
	}
	if (this->resultTransformation != 0) { // might be unused
		delete this->resultTransformation;
	}
}

void IterativeClosestPoint::match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation) {
	/*
	 * Important note:
	 * This method does not work on the member variables model and data, to prevent a mixture between the
	 * stateless (IIterativeClosestPoint) and statefull (IIterativeClosestPointDatailed) interface
	 *
	 */
	assert(assigner != 0); //check if algorithms are setup
	assert(estimator != 0);
	assert(model != 0); // check input parameters
	assert(data != 0);

	double error = 0.0;
	double previousError = 0.0;
	double previousPreviousError = 0.0;

	IHomogeneousMatrix44* tmpResultTransformation = new HomogeneousMatrix44();
	std::vector<CorrespondencePoint3DPair>* pointPairs = new std::vector<CorrespondencePoint3DPair>();

	/* perform generic ICP */
	for (int i = 0; i < maxIterations; ++i) {
		previousPreviousError = previousError;
		previousError = error;

		/* find closest points */
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
	delete pointPairs;
	delete tmpResultTransformation;

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


void IterativeClosestPoint::setData(PointCloud3D* data) {
	this->data = data;
}

void IterativeClosestPoint::setModel(PointCloud3D* model) {
	this->model = model;
}

PointCloud3D* IterativeClosestPoint::getData() {
	return this->data;
}

PointCloud3D* IterativeClosestPoint::getModel() {
	return this->model;
}

double IterativeClosestPoint::performNextIteration() {
	assert(assigner != 0); //check if algorithms are setup
	assert(estimator != 0);
	assert(this->model != 0); // check if data is set
	assert(this->data != 0);

	double error;
	if (this->intermadiateTransformation == 0) { // do only once
			this->intermadiateTransformation = new HomogeneousMatrix44();
	}
	if (this->resultTransformation == 0) { // do only once
			this->resultTransformation = new HomogeneousMatrix44();
	}
	std::vector<CorrespondencePoint3DPair>* pointPairs = new std::vector<CorrespondencePoint3DPair>();

	/*
	 * perform one ICP iteration:
	 */

	/* find closest points */
	assigner->createNearestNeighborCorrespondence(this->model, this->data, pointPairs);

	/* estimate transformation */
	error = estimator->estimateTransformation(pointPairs, this->intermadiateTransformation);
	//cout << "Estimated transformation: " << endl  << *tmpResultTransformation; //DBG output
	*(this->resultTransformation) = *((*(this->resultTransformation)) * (*(this->intermadiateTransformation))); // accumulate transformations

	/* perform transformation on data point cloud */
	this->data->homogeneousTransformation(this->intermadiateTransformation);

	delete pointPairs;
	return error;
}

IHomogeneousMatrix44*  IterativeClosestPoint::getLastEstimatedTransformation() {
	return this->intermadiateTransformation;
}

IHomogeneousMatrix44*  IterativeClosestPoint::getAccumulatedTransfomation() {
	return this->resultTransformation;
}

}

/* EOF */
