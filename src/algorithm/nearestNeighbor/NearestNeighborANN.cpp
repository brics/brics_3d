/**
 * @file 
 * NearestNeighborANN.cpp
 *
 * @date: Jan 12, 2010
 * @author: sblume
 */

#include "NearestNeighborANN.h"
#include <assert.h>
#include <stdexcept>

using std::runtime_error;

namespace BRICS_3D {

NearestNeighborANN::NearestNeighborANN() {
	this->dimension = -1;
	this->maxDistance = -1; //default = disable
	k = 1;
	eps	= 0;
	maxPts = 1000;

	dataPoints = 0;
	kdTree = 0;

}

NearestNeighborANN::~NearestNeighborANN() {
	if (kdTree != 0) {
		    delete kdTree;
	}
	if (dataPoints != 0) {
			annDeallocPts(dataPoints);
	}
	annClose();									// done with ANN
}

void NearestNeighborANN::setData(vector<vector<float> >* data) {

}

void NearestNeighborANN::setData(vector<vector<double> >* data) {

}

void NearestNeighborANN::setData(PointCloud3D* data) {
	assert(data != 0);

	if (kdTree != 0) {
		    delete kdTree;
	}
	if (dataPoints != 0) {
			annDeallocPts(dataPoints);
	}

	dimension = 3; //we work with a 3D points...
	dataPoints = annAllocPts(data->getSize(), dimension);			// allocate data points
	int nPts = static_cast<int>(data->getSize());									// read data points

	/* fill in the data int the ANN specific representation */
	for (int i = 0; i < static_cast<int>(data->getSize()); ++i) {
		dataPoints[i][0] = static_cast<ANNcoord>( (*data->getPointCloud())[i].getX() );
		dataPoints[i][1] = static_cast<ANNcoord>( (*data->getPointCloud())[i].getY() );
		dataPoints[i][2] = static_cast<ANNcoord>( (*data->getPointCloud())[i].getZ() );
	}

	kdTree = new ANNkd_tree(					// build search structure
					dataPoints,					// the data points
					nPts,						// number of points
					dimension);					// dimension of space


}

int NearestNeighborANN::findNearestNeigbor(vector<float>* query) {
	return -1;
}

int NearestNeighborANN::findNearestNeigbor(vector<double>* query) {
	return -1;
}

void NearestNeighborANN::findNearestNeigbor(Point3D* query, std::vector<int>* resultIndices, unsigned int k) {
	assert (query != 0);
	assert (resultIndices != 0);
	assert (dimension == 3);

	if (static_cast<int>(k) > kdTree->nPoints()) {
		throw runtime_error("Number of neighbors k is bigger than the amount of data points.");
	}

	this->k = static_cast<int>(k);
	resultIndices->clear();

	nnIndex = new ANNidx[k];					// allocate near neigh indices
	distances = new ANNdist[k];					// allocate near neighbor distances
	queryPoint = annAllocPt(dimension);			// allocate query point

	queryPoint[0] = static_cast<ANNcoord>( query->getX() );
	queryPoint[1] = static_cast<ANNcoord>( query->getY() );
	queryPoint[2] = static_cast<ANNcoord>( query->getZ() );

	kdTree->annkSearch(						// search
			queryPoint,						// query point
			this->k,							// number of near neighbors
			nnIndex,						// nearest neighbors (returned)
			distances,						// distance (returned)
			eps);							// error bound

//	std::cout << "\tNN:\tIndex\tDistance\n";
//	for (int i = 0; i < k; i++) {			// print summary
//		distances[i] = sqrt(distances[i]);			// unsquare distance
//		std::cout << "\t" << i << "\t" << nnIndex[i] << "\t" << distances[i] << "\n";
//	}

	BRICS_3D::Coordinate resultDistance; //distance has same data-type as Coordinate, although the meaning is different TODO: global distance typedef?
	int resultIndex;
	for (int i = 0; i < this->k; i++) {
		resultDistance = static_cast<BRICS_3D::Coordinate>(sqrt(distances[i]));	//unsquare distance
		resultIndex = nnIndex[i];
		if (resultDistance <= maxDistance || maxDistance < 0.0) { //if max distance is < 0 then the distance should have no influence
			resultIndices->push_back(resultIndex);
		}
	}



	annDeallocPt(queryPoint);					// clean things up
    delete [] nnIndex;
    delete [] distances;

}

}

/* EOF */
