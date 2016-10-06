/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2015, KU Leuven
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

#ifndef LODCALCULATOR_H_
#define LODCALCULATOR_H_

#include "Shape.h"
#include "brics_3d/algorithm/featureExtraction/DensityExtractor.h"

namespace brics_3d {
namespace rsg {

/**
 * @brief Calcultates the LOD of a Shape
 */
class LODCalculator {
public:
	LODCalculator();
	virtual ~LODCalculator();

	bool calculateLOD(Shape::ShapePtr shape, double& lod);

private:

	 DensityExtractor pointCloudLod;
};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* LODCALCULATOR_H_ */

/* EOF */
