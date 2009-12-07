/**
 * @file 
 * IterativeClosestPoint.cpp
 *
 * @date: Dec 3, 2009
 * @author: sblume
 */

#include "IterativeClosestPoint.h"

namespace BRICS_3D {

IterativeClosestPoint::IterativeClosestPoint() {


}

IterativeClosestPoint::~IterativeClosestPoint() {

}

void IterativeClosestPoint::match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation, int maxIterations) {

	/* perform generic ICP */
	for (int i = 0; i < maxIterations; ++i) {

		/* find closest points */
		std::vector<CorrespondencePoint3DPair>* pointPairs;
		assigner->createNearestNeighborCorrespondence(model, data, pointPairs);

		/* estimate transformation */
		estimator->estimateTransformation(model, data, pointPairs, resultTransformation);

		/* perform transformation on data point cloud */
		data->homogeneousTransformation(resultTransformation);

		/* stop if error is below convergence threshold */
		if (false) { //TODO
			break;
		}
	}
}

void IterativeClosestPoint::match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation, IHomogeneousMatrix44* initalEstimate, int maxIterations) {

}

}

/* EOF */
