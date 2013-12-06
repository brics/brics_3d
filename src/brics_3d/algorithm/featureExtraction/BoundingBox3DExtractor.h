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

#ifndef BRICS_3D_BOUNDINGBOX3DEXTRACTOR_H_
#define BRICS_3D_BOUNDINGBOX3DEXTRACTOR_H_

#include "brics_3d/algorithm/featureExtraction/PCA.h"
#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/Vector3D.h"
#include "brics_3d/core/IHomogeneousMatrix44.h"
#include "brics_3d/core/IPoint3DIterator.h"


namespace brics_3d {

/**
 * @brief Calculate a bounding box for a point cloud.
 * @ingroup featureExtraction
 */
class BoundingBox3DExtractor {
public:

	/**
	 * Standard constructor
	 */
	BoundingBox3DExtractor();

	/**
	 * Standard destructor.
	 */
	virtual ~BoundingBox3DExtractor();

	/**
	 * @brief Compute an axis aligned bounding box.
	 * @param[in] inputPointCloud The point cloud whose bounding box shall be computed.
	 * @param[out] resultBoxCenter Point that represents the center of the bounding box.
	 * @param[out] resultBoxDimensions 3D vector the represents the dimensions centered around the resultBoxCenter.
	 */
	void computeBoundingBox(PointCloud3D* inputPointCloud, Point3D& resultBoxCenter, Vector3D& resultBoxDimensions);

	/**
	 * Compute an oriented bounding box.
	 * Internally a PCA will be performed to find the orientation.
	 * @param[in] inputPointCloud The point cloud whose bounding box shall be computed.
	 * @param[out] resultTransform Transform the contains the oriantation as well as the center of the bounding box.
	 * @param[out] resultBoxDimensions 3D vector the represents the dimensions centered around the center.
	 */
	void computeOrientedBoundingBox(PointCloud3D* inputPointCloud, IHomogeneousMatrix44* resultTransform, Vector3D& resultBoxDimensions);

	/// Same as above but with iterator as interface.
	void computeOrientedBoundingBox(IPoint3DIterator::IPoint3DIteratorPtr inputPointCloud, IHomogeneousMatrix44* resultTransform, Vector3D& resultBoxDimensions);


protected:

	///Lower bundariy used to find min values for x, y and z.
	Point3D lowerBound;

	///Upper bundariy used to find max values for x, y and z.
	Point3D upperBound;

	///PCA used for computing an oriented bounding box.
	PCA pcaExtractor;

};

}

#endif /* BRICS_3D_BOUNDINGBOX3DEXTRACTOR_H_ */

/* EOF */
