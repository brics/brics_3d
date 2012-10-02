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

#ifndef SIMPLEIDGENERATOR_H
#define SIMPLEIDGENERATOR_H

#include "IIdGenerator.h"

namespace brics_3d {

namespace rsg {

/**
 * @brief Implementation for IIdGenerator based on a running number.
 * @ingroup sceneGraph
 */
class SimpleIdGenerator : public IIdGenerator {

public:
	SimpleIdGenerator();
	virtual ~SimpleIdGenerator();

	unsigned int getNextValidId();
	unsigned int getRootId();
    bool removeIdFromPool(unsigned int id);

private:
	unsigned int runningNumber;
	unsigned int rootId;

};

} // namespace brics_3d::RSG

} // namespace brics_3d
#endif

/* EOF */

