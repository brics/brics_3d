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


using std::cout;
using std::endl;

namespace BRICS_3D {

NearestNeighborSTANN::NearestNeighborSTANN() {
	this->dimension = -1;
	this->maxDistance = 50.0; //this value has no specific meaning - it is just a default value

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
		tmpX = (*data->getPointCloud())[i].x;
		tmpY = (*data->getPointCloud())[i].y;
		tmpZ = (*data->getPointCloud())[i].z;
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
	assert( static_cast<unsigned int>(resultIndices.size()) == static_cast<unsigned int>(k));
	assert( static_cast<unsigned int>(resultIndices.size()) > 0);
	assert( static_cast<unsigned int>(squaredResultDistances.size()) == static_cast<unsigned int>(k));
	assert( static_cast<unsigned int>(squaredResultDistances.size()) > 0);

	if ( sqrt(squaredResultDistances[0]) <= maxDistance) {
		return static_cast<int>(resultIndices[0]);
	}
	return -1;
}

int NearestNeighborSTANN::findNearestNeigbor(Point3D* query) {
	assert (query!=0);
	assert (STANNDimension == 3);

	double tmpX = (*query).x;
	double tmpY = (*query).y;
	double tmpZ = (*query).z;


	STANNPoint queryPoint(tmpX, tmpY, tmpZ);
	resultIndices.clear();
	squaredResultDistances.clear();
	nearestNeigborHandle->ksearch(queryPoint, 1, resultIndices, squaredResultDistances); //query, k=1, result
	assert( static_cast<unsigned int>(resultIndices.size()) == static_cast<unsigned int>(k));
	assert( static_cast<unsigned int>(resultIndices.size()) > 0);
	assert( static_cast<unsigned int>(squaredResultDistances.size()) == static_cast<unsigned int>(k));
	assert( static_cast<unsigned int>(squaredResultDistances.size()) > 0);

	if ( sqrt(squaredResultDistances[0]) <= maxDistance) {
		return static_cast<int>(resultIndices[0]);
	}
	return -1;

}

}

/* EOF */
