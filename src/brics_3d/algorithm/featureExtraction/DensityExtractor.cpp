/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2014, KU Leuven
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

#include "DensityExtractor.h"
#include "BoundingBox3DExtractor.h"
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/Logger.h"

using brics_3d::Logger;

namespace brics_3d {

DensityExtractor::DensityExtractor() {

}

DensityExtractor::~DensityExtractor() {

}

Density DensityExtractor::computeDensity(brics_3d::PointCloud3D* inputPointCloud) {
	PointCloud3D::PointCloud3DPtr inputPointCloudAsSmartPtr(inputPointCloud);

	Density result = computeDensity(inputPointCloudAsSmartPtr);
	inputPointCloudAsSmartPtr.reset(); // Let the temporary smart pointer forget about its load before it goes out of scope...
	return result;
}

Density DensityExtractor::computeDensity(PointCloud3D::PointCloud3DPtr inputPointCloud) {
	PointCloud3DIterator::PointCloud3DIteratorPtr it(new PointCloud3DIterator);
	it->insert(inputPointCloud);
	return computeDensity(it);
}

Density DensityExtractor::computeDensity(IPoint3DIterator::IPoint3DIteratorPtr inputPointCloud) {
		Density result;
		result.numberOfPoints = 0;
		result.volume = 0.0;
		result.density = 0.0;

		/* Number of points */
		for (inputPointCloud->begin(); !inputPointCloud->end(); inputPointCloud->next()) {
			result.numberOfPoints++;
		}

		/* Volume */
		BoundingBox3DExtractor boundingBoxEstimator;
		HomogeneousMatrix44 dummyPose; // = new HomogeneousMatrix44(); // we do not need this, but it comes with the PCA...
		Vector3D boxDimensions;
		boundingBoxEstimator.computeOrientedBoundingBox(inputPointCloud, &dummyPose, boxDimensions);
		result.volume = boxDimensions.getX() * boxDimensions.getY() * boxDimensions.getZ();

		/* Calculate density */
		if ( std::abs(result.volume) > std::numeric_limits<double>::epsilon() ) {
			result.density = result.numberOfPoints / result.volume;
		} else {
			LOG(WARNING) << "DensityExtractor::computeDensity: Division by zero detected for densety as volume = " << result.volume ;
			result.density = 0;
		}

		return result;
}



} /* namespace brics_3d */

/* EOF */
