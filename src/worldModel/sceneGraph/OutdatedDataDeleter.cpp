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

#include "OutdatedDataDeleter.h"

#include "core/Logger.h"

namespace BRICS_3D {

namespace RSG {

OutdatedDataDeleter::OutdatedDataDeleter() {
	performAutomaticHistoryUpdates = true;
	minHistoryLength = 1;
}

OutdatedDataDeleter::~OutdatedDataDeleter() {

}

void OutdatedDataDeleter::visit(Node* node){
	//nothing to delete (yet)
}

void OutdatedDataDeleter::visit(Group* node){
	//nothing to delete (yet)
}

void OutdatedDataDeleter::visit(Transform* node){

	if (performAutomaticHistoryUpdates) {
		/* optionally update history/cache */
		node->deleteOutdatedTransforms(TimeStamp(timer.getCurrentTime()));
	}

	/* check if node is outdated */
	if (node->getCurrentHistoryLenght() < minHistoryLength) {

		/* delete _this_ from the graph by deleting all handles that the parents have */
		if (node->getNumberOfParents() == 0) { // oops we are trying to delete the root node, but this one has no parents...
			LOG(WARNING) << "OutdatedDataDeleter: The root node with ID " << node->getId() << " cannot be deleted.";
			return;
		}
		LOG(DEBUG) << "Deleting Transform node with ID " << node->getId();

		/*
		 * so we found the handle to the current node; now we will invoke
		 * the according delete function for every parent
		 */
		while (node->getNumberOfParents() > 0) { //NOTE: node->getNumberOfParents() will decrease within every iteration...
			unsigned int i = 0;
			unsigned int childIndex = 0;
			RSG::Node* parentNode;
			parentNode = node->getParent(i);
			Group* parentGroup =  dynamic_cast<Group*>(parentNode);
			if (parentGroup != 0 ) {
				//parentGroup->removeChild(node); //TODO: shared count will/might go to 0...
				childIndex = parentGroup->getChildIndex(node);
				parentGroup->removeChildren(childIndex);
			} else {
				assert(false); // actually parents need to be groups otherwise sth. really went wrong
			}
		}

		//delete all (potential) child references
		node->removeChildren(0, node->getNumberOfChildren());
	}

	/*
	 * TODO how to inform SceneGraphFacade for its lookup table? -> maybe a more custom traverser?
	 * Actually it just has to invoke the facads delete function (as long as IDs are consistent)
	 */
}

void OutdatedDataDeleter::visit(GeometricNode* node){
	//nothing to delete (yet)
}

bool OutdatedDataDeleter::getPerformAutomaticHistoryUpdates() const {
    return performAutomaticHistoryUpdates;
}

void OutdatedDataDeleter::setPerformAutomaticHistoryUpdates(bool performAutomaticHistoryUpdates) {
    this->performAutomaticHistoryUpdates = performAutomaticHistoryUpdates;
}

unsigned int OutdatedDataDeleter::getMinHistoryLength() const {
    return minHistoryLength;
}

void OutdatedDataDeleter::setMinHistoryLength(unsigned int minHistoryLength) {
    this->minHistoryLength = minHistoryLength;
}

}

}

/* EOF */
