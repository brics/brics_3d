/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2011, GPS GmbH
 *
 * Author: Pinaki Sunil Banerjee
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

#ifndef RGBCOLORBASEDEUCLIDEANCLUSTERING_H_
#define RGBCOLORBASEDEUCLIDEANCLUSTERING_H_

#include "algorithm/segmentation/ISegmentation.h"



#include <iostream>

namespace BRICS_3D {

class RGBColorBasedEuclideanClustering : public ISegmentation{
public:
	RGBColorBasedEuclideanClustering();
	virtual ~RGBColorBasedEuclideanClustering();

	int segment();
};

} /* namespace BRICS_3D */
#endif /* RGBCOLORBASEDEUCLIDEANCLUSTERING_H_ */
