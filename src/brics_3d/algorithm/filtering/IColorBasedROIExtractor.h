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

#ifndef BRICS_3D_ICOLORREGIONEXTRACTOR_H_
#define BRICS_3D_ICOLORREGIONEXTRACTOR_H_

#include "brics_3d/core/ColoredPointCloud3D.h"

#include <stdlib.h>
#include <math.h>

namespace brics_3d {

/**
 * @brief Abstract interface for color-based extractions point cloud subsets
 * @ingroup filtering
 */
class IColorBasedROIExtractor {
public:

	/**
	 * Extracts subset of input point cloud based on color-properties
	 * @param in_cloud Input pointcloud (Colored)
	 * @param out_cloud Extracted Subset (Color Information Discarded)
	 */
	virtual void extractColorBasedROI(brics_3d::ColoredPointCloud3D *in_cloud, brics_3d::ColoredPointCloud3D *out_cloud)=0;



	/**
	 * Extracts subset of input point cloud based on color-properties
	 * @param in_cloud Input pointcloud (Colored)
	 * @param out_cloud Extracted Subset (Color Information Discarded)
	 */
	virtual void extractColorBasedROI(brics_3d::ColoredPointCloud3D *in_cloud, brics_3d::PointCloud3D *out_cloud)=0;
};

}

#endif /* BRICS_3D_ICOLORREGIONEXTRACTOR_H_ */
