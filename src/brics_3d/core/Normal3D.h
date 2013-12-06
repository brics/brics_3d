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

#ifndef BRICS_3D_NORMAL3D_H_
#define BRICS_3D_NORMAL3D_H_

#include "brics_3d/core/Vector3D.h"

/**
 * @namespace brics_3d
 */
namespace brics_3d {

class Normal3D : public Vector3D {
public:
	Normal3D();
	virtual ~Normal3D();

	/**
	 * @brief Constructor with full initialization
	 * @param x X coordinate in Cartesian system
	 * @param y Y coordinate in Cartesian system
	 * @param z Z coordinate in Cartesian system (height)
	 */
	Normal3D(Coordinate x, Coordinate y, Coordinate z);

	/**
	 * @brief Copy constructor
	 * @param[in] point Pointer to point that will be copied
	 */
	Normal3D(Normal3D* normal);

	/**
	 * @brief Applies a homogeneous transformation matrix to the normal. The translational part will be ignored.
	 *
	 * @param[in] transformation The homogeneous transformation matrix that will be applied
	 */
	virtual void homogeneousTransformation(IHomogeneousMatrix44 *transformation);


	/**
	 * @brief Overridden assign operator
	 * @param point Reference to right operand
	 * @return Reference to left operand
	 */
	virtual Normal3D& operator=(const Normal3D &normal);
};

}

#endif /* BRICS_3D_NORMAL3D_H_ */

/* EOF */
