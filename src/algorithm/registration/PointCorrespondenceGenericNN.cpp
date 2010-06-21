/**
 * @file 
 * PointCorrespondenceGenericNN.cpp
 *
 * @date: Jan 6, 2010
 * @author: sblume
 */

#include "PointCorrespondenceGenericNN.h"

#include <assert.h>

namespace BRICS_3D {

PointCorrespondenceGenericNN::PointCorrespondenceGenericNN() {
	this->nearestNeighborAlgorithm = 0;

}

PointCorrespondenceGenericNN::PointCorrespondenceGenericNN(INearestPoint3DNeighbor* nearestNeighborAlgorithm) {
    this->nearestNeighborAlgorithm = nearestNeighborAlgorithm;
}

PointCorrespondenceGenericNN::~PointCorrespondenceGenericNN() {
	if (nearestNeighborAlgorithm != 0) {
		delete nearestNeighborAlgorithm;
	}
}

void PointCorrespondenceGenericNN::createNearestNeighborCorrespondence(PointCloud3D* pointCloud1,
		PointCloud3D* pointCloud2, std::vector<CorrespondencePoint3DPair>* resultPointPairs) {

	assert(nearestNeighborAlgorithm != 0);  // check if algorithm is set up
	assert(pointCloud1 != 0);  // check input parameters
	assert(pointCloud2 != 0);
	assert(resultPointPairs != 0);

	resultPointPairs->clear();

	/* prepare data */
	nearestNeighborAlgorithm->setData(pointCloud1);

	/* search for each point in pointCloud2 */
	vector<int> resultIndices;
	int resultIndex;
	int k = 1; //only the nearest neighbor is considered

	for (unsigned int i = 0; i < pointCloud2->getSize(); i++) {

		nearestNeighborAlgorithm->findNearestNeighbor( &(*pointCloud2->getPointCloud())[i], &resultIndices, k);

		if (resultIndices.size() > 0) {
			resultIndex = resultIndices[0];
			assert (resultIndex < static_cast<int>(pointCloud1->getSize())); //plausibility check if result is in range

			Point3D firstPoint = Point3D (&((*pointCloud1->getPointCloud())[resultIndex]));
			Point3D secondPoint = Point3D (&((*pointCloud2->getPointCloud())[i]));

			CorrespondencePoint3DPair foundPair(firstPoint, secondPoint);
			resultPointPairs->push_back(foundPair);
		}
	}
}

INearestPoint3DNeighbor* PointCorrespondenceGenericNN::getNearestNeighborAlgorithm() const {
	return nearestNeighborAlgorithm;
}

void PointCorrespondenceGenericNN::setNearestNeighborAlgorithm(INearestPoint3DNeighbor* nearestNeighborAlgorithm) {
	this->nearestNeighborAlgorithm = nearestNeighborAlgorithm;
}

}

/* EOF */
