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

#ifndef POINTCORRESPONDANCEKDTREE_H_
#define POINTCORRESPONDANCEKDTREE_H_


#include "IPointCorrespondence.h"

namespace brics_3d {

/**
 * @ingroup registration
 * @brief Implementation of correspondence problem for points using k-d trees.
 */
class PointCorrespondenceKDTree: public brics_3d::IPointCorrespondence {
public:

	/**
	 * @brief Standard constructor
	 */
	PointCorrespondenceKDTree();

	/**
	 * @brief Standard destructor
	 */
	virtual ~PointCorrespondenceKDTree();

	void createNearestNeighborCorrespondence(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2, std::vector<CorrespondencePoint3DPair>* resultPointPairs);
};

}

#endif /* POINTCORRESPONDANCEKDTREE_H_ */

/* EOF */
