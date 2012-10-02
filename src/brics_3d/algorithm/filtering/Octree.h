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

#ifndef BRICS_3D_OCTREE_H_
#define BRICS_3D_OCTREE_H_

#include "brics_3d/algorithm/filtering/IOctreeReductionFilter.h"
#include "brics_3d/algorithm/filtering/IOctreePartition.h"
#include "brics_3d/algorithm/filtering/IOctreeSetup.h"

namespace brics_3d {

/**
 * @brief Implementation of the Octree component.
 * @ingroup filtering
 */
class Octree : public IOctreeReductionFilter, public IOctreePartition, public IOctreeSetup {
public:

	/**
	 * @brief Standard constructor.
	 */
	Octree();

	/**
	 * @brief Standard destructor.
	 */
	virtual ~Octree();

	void filter(PointCloud3D* originalPointCloud, PointCloud3D* resultPointCloud);

	void partitionPointCloud(PointCloud3D* pointCloud, std::vector<PointCloud3D*>* pointCloudCells);

	void setVoxelSize(double voxelSize);

	double getVoxelSize();

private:

	/// The maximum voxel size of the smallest cube in the Octree
	double voxelSize;

};

}

#endif /* BRICS_3D_OCTREE_H_ */

/* EOF */
