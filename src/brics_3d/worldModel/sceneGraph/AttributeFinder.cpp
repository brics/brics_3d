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

#include "AttributeFinder.h"
#include "brics_3d/core/Logger.h"

namespace brics_3d {

namespace rsg {

AttributeFinder::AttributeFinder() : INodeVisitor(downwards) {
	reset();
}

AttributeFinder::~AttributeFinder() {

}

void AttributeFinder::visit(Node* node) {
	assert (node != 0);
	LOG(DEBUG) << "Visiting node with ID: " << node->getId();

	/* check for duplicates (possibly causesd by _graph_ traversal) */
	for (unsigned int i = 0; i < static_cast<unsigned int>(matchingNodes.size()); ++i) {
		if (matchingNodes[i] == node) {
			return; //just stop
		}
	}

	/* check if attributes match */
	bool attributesMatch = false;
	for (unsigned int i = 0; i < static_cast<unsigned int>(queryAttributes.size()); ++i) {
		attributesMatch = false;
		if (attributeListContainsAttribute(node->getAttributes(), queryAttributes[i]) == false) {
			break;
		}
		attributesMatch = true; // returns true only when all loops are sucesfully/completely traversed
	}

	if (attributesMatch == true) {
		LOG(DEBUG) << "	Adding node as it has: ";
		for (unsigned int i = 0; i < static_cast<unsigned int>(queryAttributes.size()); ++i) {
			LOG(DEBUG) << " " << queryAttributes[i];
		}
		matchingNodes.push_back(node);
	}
}

void AttributeFinder::visit(Connection* connection) {
	this->visit(dynamic_cast<Node*>(connection)); //just feed forward to be handled as node
}

void AttributeFinder::visit(Group* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void AttributeFinder::visit(Transform* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void AttributeFinder::visit(GeometricNode* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void AttributeFinder::reset() {
	matchingNodes.clear();
}

}

}

/* EOF */
