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

#ifndef BRICS_3D_NORMALSET3D_H_
#define BRICS_3D_NORMALSET3D_H_

#include "Normal3D.h"
#include <vector>

namespace brics_3d {

/**
 * @brief Class to represent a set of 3D normals.
 */
class NormalSet3D {

public:

	/**
	 * @brief Standard constructor
	 */
	NormalSet3D();

	/**
	 * @brief Standard destructor
	 */
	virtual ~NormalSet3D();

	/**
	 * @brief Add a normal to the the normal set
	 * @param point Normal that will be added
	 */
	void addNormal(Normal3D normal);

	/**
	 * @brief Get the pointer to the normals
	 * @return Pointer to the normals
	 */
    std::vector<Normal3D>* getNormals();

    /**
     * @brief Get the number of normals in the normal set
     * @return Size of normals set (= number of stored normals)
     */
    unsigned int getSize();

protected:

	std::vector<Normal3D>* normals;
};

}

#endif /* BRICS_3D_NORMALSET3D_H_ */

/* EOF */
