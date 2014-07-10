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

#ifndef BRICS_3D_DENSITYEXTRACTOR_H_
#define BRICS_3D_DENSITYEXTRACTOR_H_

#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/PointCloud3DIterator.h"

namespace brics_3d {

/**
 * @brief Aggregated (intermediate) results.
 */
struct Density {
	double density;
	int numberOfPoints;
	double volume;
};

/**
 * @brief Extracts the density of a given point cloud.
 *
 * The density is given by the ratio of points in a given point
 * cloud to the volume of its estimated oriented bounding box.
 * The bounding box estimtion is based on PCA.
 *
 * @ingroup featureExtraction
 */
class DensityExtractor {
public:
	DensityExtractor();
	virtual ~DensityExtractor();

	Density computeDensity(brics_3d::PointCloud3D *inputPointCloud);
	Density computeDensity(PointCloud3D::PointCloud3DPtr inputPointCloud);
	Density computeDensity(IPoint3DIterator::IPoint3DIteratorPtr inputPointCloud);

};

} /* namespace brics_3d */

#endif /* BRICS_3D_DENSITYEXTRACTOR_H_ */

/* EOF */
