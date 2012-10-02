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

#ifndef BRICS_3D_CORRESPONDENCEPOINT3DPAIR_H_
#define BRICS_3D_CORRESPONDENCEPOINT3DPAIR_H_

#include "brics_3d/core/Point3D.h"

namespace brics_3d {

/**
 * @brief Class to represent a correspondence between two Point3D entities.
 *
 */
class CorrespondencePoint3DPair {
public:

	/**
	 * @brief Standard constructor
	 */
	CorrespondencePoint3DPair();

	/**
	 * @brief Constructor that initializes the corresponding points
	 * @param firstPoint Initialize the first point
	 * @param secondPoint Initialize the second point
	 */
	CorrespondencePoint3DPair(Point3D firstPoint, Point3D secondPoint);

	/**
	 * @brief Standard destructor
	 */
	virtual ~CorrespondencePoint3DPair();

	/// First point in first set
	Point3D firstPoint;

	/// Corresponding point in second set
	Point3D secondPoint;
};

}

#endif /* BRICS_3D_CORRESPONDENCEPOINT3DPAIR_H_ */

/* EOF */
