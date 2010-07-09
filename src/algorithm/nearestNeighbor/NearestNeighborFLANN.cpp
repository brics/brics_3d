/**
 * @file 
 * NearestNeighborFLANN.cpp
 *
 * @date: Jan 5, 2010
 * @author: sblume
 */

#include "NearestNeighborFLANN.h"

#include <assert.h>
#include <stdexcept>
#include <cmath>

using std::runtime_error;

namespace BRICS_3D {

NearestNeighborFLANN::NearestNeighborFLANN() {
	this->dimension = -1;
	this->maxDistance = -1; //default = disable
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
	assert(false); //TODO: implement
}

void NearestNeighborFLANN::setData(vector<vector<double> >* data) {
	assert(data != 0);

	if (index_id != 0) { // clean up if previous versions exist
		flann_free_index(index_id, &parameters);
	}

	dimension = (*data)[0].size();
	rows = data->size();
	cols = dimension;

	dataMatrix = new float[rows * cols];

	// convert data
	int matrixIndex = 0;
	for (int rowIndex = 0; rowIndex < rows; ++rowIndex) {
		for (int j = 0; j < dimension; ++j) {
			dataMatrix[matrixIndex + j] = static_cast<float> ( (*data)[rowIndex][j] );
		}
		matrixIndex += dimension;
	}

	// create underlying data structure
	index_id = flann_build_index(dataMatrix, rows, cols, &speedup, &parameters);
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
		dataMatrix[matrixIndex + 0] = static_cast<float> ((*data->getPointCloud())[rowIndex].getX());
		dataMatrix[matrixIndex + 1] = static_cast<float> ((*data->getPointCloud())[rowIndex].getY());
		dataMatrix[matrixIndex + 2] = static_cast<float> ((*data->getPointCloud())[rowIndex].getZ());
		matrixIndex += 3;
	}

	// create underlying data structure
	index_id = flann_build_index(dataMatrix, rows, cols, &speedup, &parameters);

}

void NearestNeighborFLANN::findNearestNeighbors(vector<float>* query, std::vector<int>* resultIndices, unsigned int k) {
	assert(false); //TODO: implement
}

void NearestNeighborFLANN::findNearestNeighbors(vector<double>* query, std::vector<int>* resultIndices, unsigned int k) {
	assert (query != 0);
	assert (resultIndices != 0);
//	assert (static_cast<int>(query->size()) == dimension);

	if (static_cast<int>(query->size()) != dimension) {
		throw runtime_error("Mismatch of query and data dimension.");
	}
	if (static_cast<int>(k) > this->rows) {
		throw runtime_error("Number of neighbors k is bigger than the amount of data points.");
	}

	resultIndices->clear();
	int nn = static_cast<int>(k);
	int tcount = 1;
	int* result = new int[nn];
	float* dists = new float[nn];

	float* queryData = new float[dimension]; //TODO: is there also a double version?!?
	for (int i = 0; i < dimension; ++i) {
		queryData[i] = static_cast<float>( (*query)[i] );
	}

	flann_find_nearest_neighbors_index(index_id, queryData, tcount, result, dists, nn, parameters.checks, &parameters);

	BRICS_3D::Coordinate resultDistance; //distance has same data-type as Coordinate, although the meaning is different TODO: global distance typedef?
	int resultIndex;
	for (int i = 0; i < nn; i++) {
		resultDistance = static_cast<BRICS_3D::Coordinate>(sqrt(dists[i])); //seems to return squared distance (although documentation does not suggest)
		resultIndex = result[i];
		if (resultDistance <= maxDistance || maxDistance < 0.0) { //if max distance is < 0 then the distance should have no influence
			resultIndices->push_back(resultIndex);
		}
	}

	delete[] queryData;
	delete[] dists;
	delete[] result;
}

void NearestNeighborFLANN::findNearestNeighbors(Point3D* query, std::vector<int>* resultIndices, unsigned int k) {
	assert (query != 0);
	assert (resultIndices != 0);
	assert (dimension == 3);

	if (static_cast<int>(k) > this->rows) {
		throw runtime_error("Number of neighbors k is bigger than the amount of data points.");
	}

	resultIndices->clear();
	int nn = static_cast<int>(k);
	int tcount = 1;
	int* result = new int[nn];
	float* dists = new float[nn];

	float* queryData = new float[dimension];
	queryData[0] = static_cast<float> (query->getX());
	queryData[1] = static_cast<float> (query->getY());
	queryData[2] = static_cast<float> (query->getZ());

	flann_find_nearest_neighbors_index(index_id, queryData, tcount, result, dists, nn, parameters.checks, &parameters);

	BRICS_3D::Coordinate resultDistance; //distance has same data-type as Coordinate, although the meaning is different TODO: global distance typedef?
	int resultIndex;
	for (int i = 0; i < nn; i++) {
		resultDistance = static_cast<BRICS_3D::Coordinate>(sqrt(dists[i])); //seems to return squared distance (although documentation does not suggest)
		resultIndex = result[i];
		if (resultDistance <= maxDistance || maxDistance < 0.0) { //if max distance is < 0 then the distance should have no influence
			resultIndices->push_back(resultIndex);
		}
	}

	delete[] queryData;
	delete[] dists;
	delete[] result;
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
