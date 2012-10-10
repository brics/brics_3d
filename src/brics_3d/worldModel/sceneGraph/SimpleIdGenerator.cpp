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
	rootId = 1u; //just an arbitrary choice here
	runningNumber = rootId + 1;
	idPool.clear();
	idPool.push_back(0u);
	idPool.push_back(rootId);
}

SimpleIdGenerator::~SimpleIdGenerator(){

}

unsigned int SimpleIdGenerator::getNextValidId(){
	unsigned int NextValidId = runningNumber++;
	removeIdFromPool(NextValidId);
	return NextValidId;
}

unsigned int SimpleIdGenerator::getRootId(){
	return rootId;
}

bool SimpleIdGenerator::removeIdFromPool(unsigned int id) {
	std::vector<unsigned int>::iterator lookUpResult = 	std::find(idPool.begin(), idPool.end(), id);
//	if (id < runningNumber) {
	if (lookUpResult != idPool.end()) {
		return false;
	}
	idPool.push_back(id);
	runningNumber = id + 1; //Generated IDs might not be continous, but that does not really matters...
	return true;
}

} // namespace brics_3d::RSG

} // namespace brics_3d

/* EOF */

