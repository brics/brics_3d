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

#ifndef IPOINTCORRESPONDENCE_H_
#define IPOINTCORRESPONDENCE_H_

#include "core/PointCloud3D.h"
//#include "core/CorrespondenceIndexPair.h"
#include "core/CorrespondencePoint3DPair.h"

#include <vector>

namespace brics_3d {

/**
 * @ingroup registration
 * @brief Abstract interface for algorithms that solve the correspondence problem for points
 */
class IPointCorrespondence {
public:

	/**
	 * @brief Standard constructor
	 */
	IPointCorrespondence(){};

	/**
	 * @brief Standard destructor
	 */
	virtual ~IPointCorrespondence(){};

	/**
	 * @brief Establishes a point to point correspondence with nearest neighborhood search
	 * @param[in] pointCloud1 Pointer to first point cloud
	 * @param[in] pointCloud2 Pointer to second point cloud
	 * @param[out] resultPointPairs Pointer where to store the resulting point correspondences. If no correspondences are found, this vector will be empty.
	 */
	virtual void createNearestNeighborCorrespondence(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2, std::vector<CorrespondencePoint3DPair>* resultPointPairs) = 0;

};

}

#endif /* IPOINTCORRESPONDENCE_H_ */

/* EOF */
