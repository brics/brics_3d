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

#ifndef POINT3DINDEXPAIR_H_
#define POINT3DINDEXPAIR_H_


namespace BRICS_3D {

/**
 * @brief Class to represent a correspondence between two entities e.g points by their respective index of thier set.
 *
 * Correspondences could be e.g. between two landmarks or Point3D entities which are the nearest neighbors of two point clouds.
 * The correspondence is represented by storing the first index of the first point cloud and the corresponding index of the other point cloud.
 */
class CorrespondenceIndexPair {

public:

	/**
	 * @brief Standard constructor
	 */
	CorrespondenceIndexPair();

	/**
	 * @brief Constructor that initializes the correspondence indices
	 * @param firstIndex Initialize index of first entity in first set
	 * @param secondIndex Initialize index of corresponding entity in second set
	 */
	CorrespondenceIndexPair(unsigned int firstIndex, unsigned int secondIndex);

	/**
	 * @brief Standard destructor
	 */
	virtual ~CorrespondenceIndexPair();

	/// Index of first entity in first set
	unsigned int firstIndex;

	/// Index of corresponding entity in second set
	unsigned int secondIndex;
};

}

#endif /* POINT3DINDEXPAIR_H_ */

/* EOF */
