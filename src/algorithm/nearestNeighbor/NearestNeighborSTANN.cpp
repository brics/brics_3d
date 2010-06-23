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

	nearestPoint3DNeigborHandle = 0;
	nearestNeigborHandle = 0;

	points3D = new vector<STANNPoint3D>();
	points = new vector<STANNPoint>;
	resultIndices = new vector<long unsigned int>();
	squaredResultDistances = new vector <double>();


}

NearestNeighborSTANN::~NearestNeighborSTANN() {
	if (nearestPoint3DNeigborHandle != 0) {
		delete nearestPoint3DNeigborHandle;
	}
	if (nearestNeigborHandle != 0) {
		delete nearestNeigborHandle;
	}

	points3D->clear();
	points->clear();
	resultIndices->clear();
	squaredResultDistances->clear();

	delete points3D;
	delete points;
	delete resultIndices;
	delete squaredResultDistances;
}

void NearestNeighborSTANN::setData(vector<vector<float> >* data) {
	assert(false); //TODO: implement
}

void NearestNeighborSTANN::setData(vector<vector<double> >* data) {
	assert (data != 0);

	if ((*data)[0].size() != STANNDimension) {
		throw runtime_error("Mismatch of data and preconfigured dimension. Please set constant \"STANNDimension\" accordingly. ");
	}

	if (nearestNeigborHandle != 0) {
		delete nearestNeigborHandle; // clean up old stuff first
	}

	dimension = (*data)[0].size();
	points->clear();

	/* fill in the data into the STANN specific representation */
	for (int i = 0; i < static_cast<int>(data->size()); ++i) {
		STANNPoint tmpPoint;
		for (int j = 0; j < dimension; ++j) {
			tmpPoint[j] = (*data)[i][j];
		}
		points->push_back(tmpPoint);
	}

	nearestNeigborHandle = new sfcnn<STANNPoint, STANNDimension, double> (&(*points)[0], static_cast<int>(data->size())); // create morton ordering
}

void NearestNeighborSTANN::setData(PointCloud3D* data) {
	assert (data != 0);
	assert(STANNPoint3DDimension == 3);

	this->dimension = 3;

	if (nearestPoint3DNeigborHandle != 0) {
		delete nearestPoint3DNeigborHandle; // clean up old stuff first
		points3D->clear();
	}

	double tmpX = 0.0;
	double tmpY = 0.0;
	double tmpZ = 0.0;

	for (int i = 0; i < static_cast<int>(data->getSize()); ++i) {
		tmpX = (*data->getPointCloud())[i].getX();
		tmpY = (*data->getPointCloud())[i].getY();
		tmpZ = (*data->getPointCloud())[i].getZ();
		STANNPoint3D tmpPoint(tmpX, tmpY, tmpZ);
		points3D->push_back(tmpPoint);
//		cout << tmpPoint;
	}

	nearestPoint3DNeigborHandle = new sfcnn<STANNPoint3D, STANNPoint3DDimension, double> (&(*points3D)[0], static_cast<int>(data->getSize())); // create morton ordering

}

void NearestNeighborSTANN::findNearestNeighbors(vector<float>* query, std::vector<int>* resultIndices, unsigned int k) {
	assert(false); //TODO: implement
}

void NearestNeighborSTANN::findNearestNeighbors(vector<double>* query, std::vector<int>* resultIndices, unsigned int k) {
	assert (query != 0);
	assert (resultIndices != 0);

	if (static_cast<int>(query->size()) != dimension) {
		throw runtime_error("Mismatch of query and data dimension.");
	}
	if (static_cast<unsigned int>(k) > this->points->size()) {
		throw runtime_error("Number of neighbors k is bigger than the amount of data points.");
	}

	STANNPoint queryPoint;
	for (int i = 0; i < dimension; ++i) {
		queryPoint[i] = (*query)[i];
	}

	squaredResultDistances->clear();
	this->resultIndices->clear(); //clear internal vector
	resultIndices->clear(); //result vector

	nearestNeigborHandle->ksearch(queryPoint, k, *(this->resultIndices), *squaredResultDistances);
	assert( static_cast<unsigned int>(this->resultIndices->size()) == static_cast<unsigned int>(k));
	assert( static_cast<unsigned int>(this->resultIndices->size()) > 0);
	assert( static_cast<unsigned int>(squaredResultDistances->size()) == static_cast<unsigned int>(k));
	assert( static_cast<unsigned int>(squaredResultDistances->size()) > 0);

	BRICS_3D::Coordinate resultDistance; //distance has same data-type as Coordinate, although the meaning is different
	int resultIndex;
	for (int i = 0; i < static_cast<int>(k); i++) {
		resultDistance = static_cast<BRICS_3D::Coordinate>(sqrt((*squaredResultDistances)[i])); //seems to return squared distance (although documentation does not suggest)
		resultIndex = static_cast<int>((*(this->resultIndices))[i]);
		if (resultDistance <= maxDistance || maxDistance < 0.0) { //if max distance is < 0 then the distance should have no influence
			resultIndices->push_back(resultIndex);
		}
	}

}

void NearestNeighborSTANN::findNearestNeighbors(Point3D* query, std::vector<int>* resultIndices, unsigned int k) {
	assert (query != 0);
	assert (resultIndices != 0);
	assert (STANNPoint3DDimension == 3);

	if (static_cast<unsigned int>(k) > this->points3D->size()) {
		throw runtime_error("Number of neighbors k is bigger than the amount of data points.");
	}

	resultIndices->clear();
	double tmpX = (*query).getX();
	double tmpY = (*query).getY();
	double tmpZ = (*query).getZ();


	STANNPoint3D queryPoint(tmpX, tmpY, tmpZ);
	this->resultIndices->clear();
	squaredResultDistances->clear();
	nearestPoint3DNeigborHandle->ksearch(queryPoint, k, *(this->resultIndices), *squaredResultDistances);
	assert( static_cast<unsigned int>(this->resultIndices->size()) == static_cast<unsigned int>(k));
	assert( static_cast<unsigned int>(this->resultIndices->size()) > 0);
	assert( static_cast<unsigned int>(squaredResultDistances->size()) == static_cast<unsigned int>(k));
	assert( static_cast<unsigned int>(squaredResultDistances->size()) > 0);

	BRICS_3D::Coordinate resultDistance; //distance has same data-type as Coordinate, although the meaning is different
	int resultIndex;
	for (int i = 0; i < static_cast<int>(k); i++) {
		resultDistance = static_cast<BRICS_3D::Coordinate>(sqrt((*squaredResultDistances)[i])); //seems to return squared distance (although documentation does not suggest)
		resultIndex = static_cast<int>((*(this->resultIndices))[i]);
		if (resultDistance <= maxDistance || maxDistance < 0.0) { //if max distance is < 0 then the distance should have no influence
			resultIndices->push_back(resultIndex);
		}
	}
}

}

/* EOF */
