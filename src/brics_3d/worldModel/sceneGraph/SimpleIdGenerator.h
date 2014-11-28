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

#ifndef RSG_SIMPLEIDGENERATOR_H
#define RSG_SIMPLEIDGENERATOR_H

#include "IIdGenerator.h"
#include <vector>

namespace brics_3d {

namespace rsg {

/**
 * @brief Implementation for IIdGenerator based on a running number.
 * @ingroup sceneGraph
 */
class SimpleIdGenerator : public IIdGenerator {

public:
	SimpleIdGenerator();
	SimpleIdGenerator(Id rootId);
	virtual ~SimpleIdGenerator();

	Id getNextValidId();
	Id getRootId();
    bool removeIdFromPool(Id id);

private:
	Id rootId;
	unsigned int runningNumber;

	std::vector<Id> idPool;
};

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

