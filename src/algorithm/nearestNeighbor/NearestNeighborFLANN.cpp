/**
 * @file 
 * NearestNeighborFLANN.cpp
 *
 * @date: Jan 5, 2010
 * @author: sblume
 */

#include "NearestNeighborFLANN.h"

#include <assert.h>

namespace BRICS_3D {

NearestNeighborFLANN::NearestNeighborFLANN() {
	this->dimension = -1;
	this->maxDistance = 50.0; //this value has no specific meaning - it is just a default value
	this->dataMatrix = NULL;
	this->index_id = NULL;

	this->parameters.log_level = LOG_NONE;
	this->parameters.log_destination = NULL;

	this->parameters.algorithm = KDTREE;
	this->parameters.checks = 32;
	this->parameters.trees = 8;
	this->parameters.branching = 32;
	this->parameters.iterations = 7;
	this->parameters.target_precision = -1;
}

NearestNeighborFLANN::~NearestNeighborFLANN() {
	flann_free_index(index_id, &parameters);
}

void NearestNeighborFLANN::setData(vector<vector<float> >* data) {

}
void NearestNeighborFLANN::setData(vector<vector<double> >* data) {

}
void NearestNeighborFLANN::setData(PointCloud3D* data) {
	assert(data != 0);

	if (index_id != 0) { // clean up if previous versions exist
		flann_free_index(index_id, &parameters);
	}

	dimension = 3; //we work with a 3D points...
	rows = data->getSize();
	cols = dimension;

	dataMatrix = new float[rows * cols];

	// convert data
	int matrixIndex = 0;
	for (int rowIndex = 0; rowIndex < rows; ++rowIndex) {
		dataMatrix[matrixIndex + 0] = static_cast<float> ((*data->getPointCloud())[rowIndex].x);
		dataMatrix[matrixIndex + 1] = static_cast<float> ((*data->getPointCloud())[rowIndex].y);
		dataMatrix[matrixIndex + 2] = static_cast<float> ((*data->getPointCloud())[rowIndex].z);
		matrixIndex += 3;
	}

	// create underlying data structure
	index_id = flann_build_index(dataMatrix, rows, cols, &speedup, &parameters);

}

int NearestNeighborFLANN::findNearestNeigbor(vector<float>* query) {
	return -1;
}
int NearestNeighborFLANN::findNearestNeigbor(vector<double>* query) {
	return -1;
}
int NearestNeighborFLANN::findNearestNeigbor(Point3D* query) {
	assert(this->dataMatrix != 0);
	assert(this->dimension = 3);

	int nn = 1; // we only want the first nearest neighbor
	int tcount = 1;

	float queryData[dimension];
	queryData[0] = static_cast<float> (query->x);
	queryData[1] = static_cast<float> (query->y);
	queryData[2] = static_cast<float> (query->z);

	int result[1];
	float dists[1];

	flann_find_nearest_neighbors_index(index_id, queryData, tcount, result, dists, nn, parameters.checks, &parameters);

	if (dists[0] <= maxDistance) {
		return result[0];
	}
	return -1;
}

FLANNParameters NearestNeighborFLANN::getParameters() const {
	return parameters;
}

void NearestNeighborFLANN::setParameters(FLANNParameters p) {
	this->parameters = p;
}

float NearestNeighborFLANN::getSpeedup() const {
	return speedup;
}

}

/* EOF */
