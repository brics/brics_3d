/**
 * @file 
 * NearestNeighborSTANN.cpp
 *
 * @date: Jan 11, 2010
 * @author: sblume
 */

#include "NearestNeighborSTANN.h"
#include <assert.h>
#include <cmath>
#include <stdexcept>

using std::cout;
using std::endl;
using std::runtime_error;

namespace BRICS_3D {

NearestNeighborSTANN::NearestNeighborSTANN() {
	this->dimension = -1;
	this->maxDistance = -1; //default = disable

	nearestNeigborHandle = 0;
	points.clear();

}

NearestNeighborSTANN::~NearestNeighborSTANN() {
	if (nearestNeigborHandle != 0) {
		delete nearestNeigborHandle;
	}
	points.clear();
}

void NearestNeighborSTANN::setData(vector<vector<float> >* data) {

}

void NearestNeighborSTANN::setData(vector<vector<double> >* data) {
	assert (data != 0);
	assert( (*data)[0].size() == STANNDimension);

	if (nearestNeigborHandle != 0) {
		delete nearestNeigborHandle; // clean up old stuff first
		points.clear();
	}

	double tmpX = 0.0;
	double tmpY = 0.0;
	double tmpZ = 0.0;

	for (int i = 0; i < static_cast<int>(data->size()); ++i) {
		tmpX = ((*data)[i])[0];
		tmpY = ((*data)[i])[1];
		tmpZ = ((*data)[i])[2];
		STANNPoint tmpPoint(tmpX, tmpY, tmpZ);
		points.push_back(tmpPoint);
//		cout << tmpPoint;
	}

	nearestNeigborHandle = new sfcnn<STANNPoint, STANNDimension, double> (&points[0], static_cast<int>(data->size())); // create morton ordering
}

void NearestNeighborSTANN::setData(PointCloud3D* data) {
	assert (data != 0);
	assert(STANNDimension == 3);

	this->dimension = 3;

	if (nearestNeigborHandle != 0) {
		delete nearestNeigborHandle; // clean up old stuff first
		points.clear();
	}

	double tmpX = 0.0;
	double tmpY = 0.0;
	double tmpZ = 0.0;

	for (int i = 0; i < static_cast<int>(data->getSize()); ++i) {
		tmpX = (*data->getPointCloud())[i].getX();
		tmpY = (*data->getPointCloud())[i].getY();
		tmpZ = (*data->getPointCloud())[i].getZ();
		STANNPoint tmpPoint(tmpX, tmpY, tmpZ);
		points.push_back(tmpPoint);
//		cout << tmpPoint;
	}

	nearestNeigborHandle = new sfcnn<STANNPoint, STANNDimension, double> (&points[0], static_cast<int>(data->getSize())); // create morton ordering


	//STANNPoint
}

int NearestNeighborSTANN::findNearestNeigbor(vector<float>* query) {
	return -1;
}

int NearestNeighborSTANN::findNearestNeigbor(vector<double>* query) {
	assert (query!=0);
	assert (query->size() == STANNDimension);

	double tmpX = (*query)[0];
	double tmpY = (*query)[1];
	double tmpZ = (*query)[2];


	STANNPoint queryPoint(tmpX, tmpY, tmpZ);
	resultIndices.clear();
	squaredResultDistances.clear();
	nearestNeigborHandle->ksearch(queryPoint, 1, resultIndices, squaredResultDistances); //query, k=1, result
	assert( static_cast<unsigned int>(resultIndices.size()) > 0);
	assert( static_cast<unsigned int>(squaredResultDistances.size()) > 0);

	if ( sqrt(squaredResultDistances[0]) <= maxDistance) {
		return static_cast<int>(resultIndices[0]);
	}
	return -1;
}

void NearestNeighborSTANN::findNearestNeigbor(Point3D* query, std::vector<int>* resultIndices, unsigned int k) {
	assert (query != 0);
	assert (resultIndices != 0);
	assert (STANNDimension == 3);

	if (static_cast<unsigned int>(k) > this->points.size()) {
		throw runtime_error("Number of neighbors k is bigger than the amount of data points.");
	}

	resultIndices->clear();
	double tmpX = (*query).getX();
	double tmpY = (*query).getY();
	double tmpZ = (*query).getZ();


	STANNPoint queryPoint(tmpX, tmpY, tmpZ);
	this->resultIndices.clear();
	squaredResultDistances.clear();
	nearestNeigborHandle->ksearch(queryPoint, k, this->resultIndices, squaredResultDistances); //query, k=1, result
	assert( static_cast<unsigned int>(this->resultIndices.size()) == static_cast<unsigned int>(k));
	assert( static_cast<unsigned int>(this->resultIndices.size()) > 0);
	assert( static_cast<unsigned int>(squaredResultDistances.size()) == static_cast<unsigned int>(k));
	assert( static_cast<unsigned int>(squaredResultDistances.size()) > 0);

	BRICS_3D::Coordinate resultDistance; //distance has same data-type as Coordinate, although the meaning is different
	int resultIndex;
	for (int i = 0; i < static_cast<int>(k); i++) {
		resultDistance = static_cast<BRICS_3D::Coordinate>(sqrt(squaredResultDistances[i])); //seems to return squared distance (although documentation does not suggest)
		resultIndex = static_cast<int>(this->resultIndices[i]);
		if (resultDistance <= maxDistance || maxDistance < 0.0) { //if max distance is < 0 then the distance should have no influence
			resultIndices->push_back(resultIndex);
		}
	}
}

}

/* EOF */
