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

namespace brics_3d {

namespace rsg {

SimpleIdGenerator::SimpleIdGenerator() {
	rootId = 1u; //just an arbitrary choice here
	runningNumber = rootId + 1;
}

SimpleIdGenerator::~SimpleIdGenerator(){

}

unsigned int SimpleIdGenerator::getNextValidId(){
	return runningNumber++;
}

unsigned int SimpleIdGenerator::getRootId(){
	return rootId;
}

bool SimpleIdGenerator::removeIdFromPool(unsigned int id) {
	if (id < runningNumber) {
		return false;
	}
	runningNumber = id + 1; //Generated IDs might not be continous, but that does not really matters...
	return true;
}

} // namespace brics_3d::RSG

} // namespace brics_3d

/* EOF */

