/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2014, KU Leuven
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

#ifndef RSG_UUIDGENERATOR_H_
#define RSG_UUIDGENERATOR_H_

#include "IIdGenerator.h"
#include <vector>
#include <boost/uuid/uuid_generators.hpp>

namespace brics_3d {
namespace rsg {

class UuidGenerator: public brics_3d::rsg::IIdGenerator {
public:
	UuidGenerator();
	virtual ~UuidGenerator();

	// Interface implementations
	Id getNextValidId();
	Id getRootId();
    bool removeIdFromPool(Id id);

protected:

    Id rootId;
	std::vector<Id> idPool;

	boost::uuids::random_generator idGenerator;
};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_UUIDGENERATOR_H_ */

/* EOF */
