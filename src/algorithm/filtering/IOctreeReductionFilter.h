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

#ifndef IOCTREEREDUCTIONFILTER_H_
#define IOCTREEREDUCTIONFILTER_H_

#include "core/PointCloud3D.h"
#include "IFiltering.h"

namespace BRICS_3D {

/**
 * @brief Abstract interface for the point cloud size reduction algorithm.
 *
 * This interface allows to reduce the size of a point cloud. The reduced point cloud
 * is approximated by the centers of the leaf cells of an Octree
 *
 * @ingroup filtering
 */
class IOctreeReductionFilter : public IFiltering {
public:

	/**
	 * @brief Standard constructor.
	 */
	IOctreeReductionFilter(){};

	/**
	 * @brief Standard destructor.
	 */
	virtual ~IOctreeReductionFilter(){};

	/**
	 * @brief Reduce the size of a point cloud.
	 *
	 * The behavior of the size reduction can be influenced with the parameter voxel size: IOctreeSetup::setVoxelSize()
	 *
	 * @param[in] originalPointCloud The input point cloud that will be reduced. This data will not
	 * be modified.
	 * @param[out] resultPointCloud The new point cloud with the reduced amount of points.
	 */
	virtual void filter(PointCloud3D* originalPointCloud, PointCloud3D* resultPointCloud) = 0;


};

}

#endif /* IOCTREEREDUCTIONFILTER_H_ */

/* EOF */
