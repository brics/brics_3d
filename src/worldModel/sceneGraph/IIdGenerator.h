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

#ifndef IIDGENERATOR_H
#define IIDGENERATOR_H

namespace BRICS_3D {

namespace RSG {

/**
 * @brief Abstract interface for unique ID generation.
 */
class IIdGenerator { // TODO: Should be singelton?
  public:

	IIdGenerator(){};
	virtual ~IIdGenerator(){};

    virtual unsigned int getNextValidId() = 0;
    virtual unsigned int getRootId() = 0;

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

