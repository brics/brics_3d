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

#include "OutdatedDataIdAwareDeleter.h"
#include <assert.h>

namespace BRICS_3D {

namespace RSG {

OutdatedDataIdAwareDeleter::OutdatedDataIdAwareDeleter(SceneGraphFacade* facadeHandle) {
	assert(facadeHandle != 0);
	this->facadeHandle = facadeHandle;
}

OutdatedDataIdAwareDeleter::~OutdatedDataIdAwareDeleter() {
	facadeHandle = 0; //we do not delete, as we are not the owner
}

void OutdatedDataIdAwareDeleter::doDeleteNode(Node* node) {
	assert(node != 0);
	facadeHandle->deleteNode(node->getId());
}

}

}
/* EOF */
