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

#ifndef BRICS_3D_IOCTREEPARTITION_H_
#define BRICS_3D_IOCTREEPARTITION_H_

#include "core/PointCloud3D.h"
#include <vector>

namespace brics_3d {

/**
 * @brief Abstract interface for spatial partition of point clouds.
 *
 * This interface allows to partition a point cloud into several point cloud with the help of an Octree.
 * Each leaf cell of the octree forms a partition.
 *
 * @ingroup filtering
 */
class IOctreePartition {
public:

	/**
	 * @brief Standard constructor.
	 */
	IOctreePartition(){};

	/**
	 * @brief Standard destructor.
	 */
	virtual ~IOctreePartition(){};

	/**
	 * @brief Partitions a point cloud into sub points clouds.
	 *
	 * The behavior of the subdivision can be influenced with the parameter voxel size: IOctreeSetup::setVoxelSize()
	 *
	 * @param[in] pointCloud The input point cloud that will be partitioned. This data will not
	 * be modified.
	 * @param[out] pointCloudCells Resulting vector of point clouds. Each point cloud represents one cell of the partition.
	 *
	 */
	virtual void partitionPointCloud(PointCloud3D* pointCloud, std::vector<PointCloud3D*>* pointCloudCells) = 0;
};

}

#endif /* BRICS_3D_IOCTREEPARTITION_H_ */

/* EOF */
