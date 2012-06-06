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

#ifndef BOUNDINGBOX3D_H_
#define BOUNDINGBOX3D_H_

#include "core/PointCloud3D.h"
#include "core/Vector3D.h"

namespace BRICS_3D {

/**
 * @brief Calculate a bounding box for a point cloud.
 */
class BoundingBox3DExtractor {
public:
	BoundingBox3DExtractor();
	virtual ~BoundingBox3DExtractor();

	/**
	 * @brief Compute an axis aligned bounding box.
	 * @param[in] inputPointCloud The point cloud whose bounding box shall be computed.
	 * @param[out] resultBoxCenter Point that represents the center of the buinding box
	 * @param[out] resultBoxDimensions 3D vector the represents the dimensions centered around the resultBoxCenter
	 */
	void computeBoundingBox(PointCloud3D* inputPointCloud, Point3D& resultBoxCenter, Vector3D& resultBoxDimensions);

protected:
	Point3D lowerBound;
	Point3D upperBound;
};

}

#endif /* BOUNDINGBOX3D_H_ */

/* EOF */
