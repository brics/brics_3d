/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#include "BoundingBox3DExtractor.h"
#include "Centroid3D.h"
#include "core/Logger.h"
#include "core/HomogeneousMatrix44.h"
#include <limits>

namespace brics_3d {

BoundingBox3DExtractor::BoundingBox3DExtractor() {

}

BoundingBox3DExtractor::~BoundingBox3DExtractor() {

}

void BoundingBox3DExtractor::computeBoundingBox(PointCloud3D* inputPointCloud, Point3D& resultBoxCenter, Vector3D& resultBoxDimensions) {
	lowerBound.setX(std::numeric_limits<brics_3d::Coordinate>::max());
	lowerBound.setY(std::numeric_limits<brics_3d::Coordinate>::max());
	lowerBound.setZ(std::numeric_limits<brics_3d::Coordinate>::max());
	upperBound.setX(-std::numeric_limits<brics_3d::Coordinate>::max());
	upperBound.setY(-std::numeric_limits<brics_3d::Coordinate>::max());
	upperBound.setZ(-std::numeric_limits<brics_3d::Coordinate>::max());

	/* get min/max */
	for (unsigned int i = 0; i < inputPointCloud->getSize(); ++i) {
		Point3D* currentPoint;
		currentPoint = &(*inputPointCloud->getPointCloud())[i];

		/* adjust lower bound if necessary */
		if (currentPoint->getX() <= lowerBound.getX()) {
			lowerBound.setX(currentPoint->getX());
		}
		if (currentPoint->getY() <= lowerBound.getY()) {
			lowerBound.setY(currentPoint->getY());
		}
		if (currentPoint->getZ() <= lowerBound.getZ()) {
			lowerBound.setZ(currentPoint->getZ());
		}

		/* adjust upper bound if necessary */
		if ( currentPoint->getX() >= upperBound.getX()) {
			upperBound.setX(currentPoint->getX());
		}
		if	(currentPoint->getY() >= upperBound.getY()) {
			upperBound.setY(currentPoint->getY());
		}
		if	(currentPoint->getZ() >= upperBound.getZ()) {
			upperBound.setZ(currentPoint->getZ());
		}
	}

	/* get centroid */
	Centroid3D centroidExtractor;
	Eigen::Vector3d centroid;
	centroid = centroidExtractor.computeCentroid(inputPointCloud);
	resultBoxCenter.setX(centroid[0]);
	resultBoxCenter.setY(centroid[1]);
	resultBoxCenter.setZ(centroid[2]);

	resultBoxDimensions.setX(fabs(upperBound.getX() - lowerBound.getX()));
	resultBoxDimensions.setY(fabs(upperBound.getY() - lowerBound.getY()));
	resultBoxDimensions.setZ(fabs(upperBound.getZ() - lowerBound.getZ()));

}

void BoundingBox3DExtractor::computeOrientedBoundingBox(PointCloud3D* inputPointCloud, IHomogeneousMatrix44* resultTransform, Vector3D& resultBoxDimensions) {
	assert(resultTransform != 0);

	lowerBound.setX(std::numeric_limits<brics_3d::Coordinate>::max());
	lowerBound.setY(std::numeric_limits<brics_3d::Coordinate>::max());
	lowerBound.setZ(std::numeric_limits<brics_3d::Coordinate>::max());
	upperBound.setX(-std::numeric_limits<brics_3d::Coordinate>::max());
	upperBound.setY(-std::numeric_limits<brics_3d::Coordinate>::max());
	upperBound.setZ(-std::numeric_limits<brics_3d::Coordinate>::max());

	Eigen::MatrixXd eigenvectors;
	Eigen::VectorXd eigenvalues;
	pcaExtractor.computePrincipleComponents(inputPointCloud, eigenvectors, eigenvalues); //actually we compute a centroid twice...
	pcaExtractor.computeRotationMatrix(eigenvectors, eigenvalues, resultTransform);

	/* get min/max for PCA rotated points */
	HomogeneousMatrix44* inverseRotation = new HomogeneousMatrix44();
	*inverseRotation = *(resultTransform);
	inverseRotation->inverse();
	for (unsigned int i = 0; i < inputPointCloud->getSize(); ++i) {
		Point3D tmpPoint = (*inputPointCloud->getPointCloud())[i];
		tmpPoint.homogeneousTransformation(inverseRotation); // move _all_ points to new frame
		Point3D* currentPoint = &tmpPoint;

		/* adjust lower bound if necessary */
		if (currentPoint->getX() <= lowerBound.getX()) {
			lowerBound.setX(currentPoint->getX());
		}
		if (currentPoint->getY() <= lowerBound.getY()) {
			lowerBound.setY(currentPoint->getY());
		}
		if (currentPoint->getZ() <= lowerBound.getZ()) {
			lowerBound.setZ(currentPoint->getZ());
		}

		/* adjust upper bound if necessary */
		if ( currentPoint->getX() >= upperBound.getX()) {
			upperBound.setX(currentPoint->getX());
		}
		if	(currentPoint->getY() >= upperBound.getY()) {
			upperBound.setY(currentPoint->getY());
		}
		if	(currentPoint->getZ() >= upperBound.getZ()) {
			upperBound.setZ(currentPoint->getZ());
		}
	}
	delete inverseRotation;

	/* get centroid */
	Centroid3D centroidExtractor;
	Eigen::Vector3d centroid;
	centroid = centroidExtractor.computeCentroid(inputPointCloud);

	/* as centroid as translation */
	double* matrixData;
	matrixData = resultTransform->setRawData();
	matrixData[12] = centroid[0];
	matrixData[13] = centroid[1];
	matrixData[14] = centroid[2];

	LOG(DEBUG) << "Estimated transform for oriented bounding box: "<< std::endl << *resultTransform;

	resultBoxDimensions.setX(fabs(upperBound.getX() - lowerBound.getX()));
	resultBoxDimensions.setY(fabs(upperBound.getY() - lowerBound.getY()));
	resultBoxDimensions.setZ(fabs(upperBound.getZ() - lowerBound.getZ()));

}

}

/* EOF */
