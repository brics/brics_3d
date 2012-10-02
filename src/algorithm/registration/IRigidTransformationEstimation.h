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

#ifndef BRICS_3D_IRIGIDTRANSFORMATIONESTIMATION_H_
#define BRICS_3D_IRIGIDTRANSFORMATIONESTIMATION_H_

#include "core/PointCloud3D.h"
#include "core/IHomogeneousMatrix44.h"
#include "core/CorrespondencePoint3DPair.h"

#include <vector>

namespace brics_3d {

/**
 * @ingroup registration
 * @brief Abstract interface to estimate the rigid transformation between two corresponding point clouds.
 *
 * To estimate the transformation the mean square error function is minimized.
 */
class IRigidTransformationEstimation {
public:

	/**
	 * @brief Standard constructor
	 */
	IRigidTransformationEstimation(){};

	/**
	 * @brief Standard destructor
	 */
	virtual ~IRigidTransformationEstimation(){};

	/**
	 * @brief Estimates the rigid transformation between two corresponding point clouds.
	 *
	 * @param[in] pointPairs Pointer to point correspondences between points of pointCloud1 and pointCloud2
	 * @param[out] resultTransformation Pointer to resulting rigid transformation, represented as a homogeneous matrix
	 * @return Returns the RMS point-to-point error
	 *
	 * NOTE: A function with two point clouds a correspondence representation based on CorrespondenceIndexPair might be more efficient,
	 * as the pointPairs vector does not need to be filled, but this implies that however the correspondences are established, the underlying structure is
	 * based on a vector. This assumption does not always hold e.g. the k-d tree does not necessarily store its data in a consecutive vector.
	 */
	virtual double estimateTransformation(std::vector<CorrespondencePoint3DPair>* pointPairs, IHomogeneousMatrix44* resultTransformation) = 0;

};

}

#endif /* BRICS_3D_IRIGIDTRANSFORMATIONESTIMATION_H_ */

/* EOF */
