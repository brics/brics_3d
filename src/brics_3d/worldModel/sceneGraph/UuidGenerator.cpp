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

#include "UuidGenerator.h"

namespace brics_3d {
namespace rsg {

UuidGenerator::UuidGenerator() {
	rootId = getNextValidId();
	removeIdFromPool(rootId);
	Id nullId = 0;
	removeIdFromPool(nullId);
}

UuidGenerator::~UuidGenerator() {

}

Id UuidGenerator::getNextValidId() {
	Uuid nextValidUuid;

	boost::uuids::uuid uuid = idGenerator();
	std::copy(uuid.begin(), uuid.end(), nextValidUuid.begin());

	Id nextValidId;
#ifdef NO_UUID
	nextValidId = uuidToUnsignedInt(nextValidUuid);
#else
	nextValidId = nextValidUuid;
#endif
	removeIdFromPool(nextValidId);
	return nextValidId;
}

Id UuidGenerator::getRootId() {
	return rootId;
}

bool UuidGenerator::removeIdFromPool(Id id) {
	/* this is actually not necessary as duplications are _very_ unlikely */
	std::vector<Id>::iterator lookUpResult = std::find(idPool.begin(), idPool.end(), id);
	if (lookUpResult != idPool.end()) {
		return false;
	}
	idPool.push_back(id);
	return true;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
