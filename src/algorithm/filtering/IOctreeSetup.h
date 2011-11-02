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

#ifndef IOCTREESETUP_H_
#define IOCTREESETUP_H_

/*
 *
 */
namespace BRICS_3D {

/**
 * @brief Abstract setup interface for the octree component.
 * @ingroup filtering
 */
class IOctreeSetup {
public:

	/**
	 * @brief Standard constructor.
	 */
	IOctreeSetup(){};

	/**
	 * @brief Standard destructor.
	 */
	virtual ~IOctreeSetup(){};

	/**
	 * @brief Set the size of the maximum of smallest voxel in an Octree.
	 *
	 * @param voxelSize The new voxel size.
	 * NOTE: the current implementation interprets voxelSize as the from the center to one of the cube faces.
	 */
	virtual void setVoxelSize(double voxelSize) = 0;

	/**
	 * @brief Get the current size the maximum of the smallest voxel in an Octree.
	 * @return The current size.
	 */
	virtual double getVoxelSize() = 0;
};

}

#endif /* IOCTREESETUP_H_ */

/* EOF */
