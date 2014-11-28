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

#include "SimpleIdGenerator.h"
#include <algorithm>

namespace brics_3d {

namespace rsg {

SimpleIdGenerator::SimpleIdGenerator() {
	this->rootId = 1u; //just an arbitrary choice here
	runningNumber = uuidToUnsignedInt(rootId) + 1;
	idPool.clear();
	Id nullId = 0;
	idPool.push_back(nullId);
	idPool.push_back(rootId);
}

SimpleIdGenerator::SimpleIdGenerator(Id rootId) {
	if(!rootId.isNil()) {
		this->rootId = rootId;
	} else { // fall back in case of NIL
		this->rootId = 1u; //just an arbitrary choice here
	}
	runningNumber = uuidToUnsignedInt(rootId) + 1;
	idPool.clear();
	Id nullId = 0;
	idPool.push_back(nullId);
	idPool.push_back(rootId);
}

SimpleIdGenerator::~SimpleIdGenerator(){

}

Id SimpleIdGenerator::getNextValidId(){
	Id nextValidId;
	nextValidId = runningNumber++;
	removeIdFromPool(nextValidId);
	return nextValidId;
}

Id SimpleIdGenerator::getRootId(){
	return rootId;
}

bool SimpleIdGenerator::removeIdFromPool(Id id) {
	std::vector<Id>::iterator lookUpResult = 	std::find(idPool.begin(), idPool.end(), id);
//	if (id < runningNumber) {
	if (lookUpResult != idPool.end()) {
		return false;
	}
	idPool.push_back(id);
	runningNumber = uuidToUnsignedInt(id) + 1; //Generated IDs might not be continuous, but that does not really matters...
	return true;
}

} // namespace brics_3d::RSG

} // namespace brics_3d

/* EOF */

