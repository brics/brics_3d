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

#include "SubGraphChecker.h"

namespace brics_3d {

namespace rsg {

SubGraphChecker::SubGraphChecker(Id subGraphNodeId) : INodeVisitor(upwards) {
	reset(subGraphNodeId);
}

SubGraphChecker::~SubGraphChecker() {

}

void SubGraphChecker::visit(Node* node) {
	assert (node != 0);
	//LOG(DEBUG) << "Visiting node with ID: " << node->getId();

	/* check if IDs match */
	if (node->getId() == subGraphNodeId) {
		isInSubGraph = true;
		pathCount++;
	}
}

void SubGraphChecker::visit(Group* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void SubGraphChecker::visit(Transform* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void SubGraphChecker::visit(GeometricNode* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void SubGraphChecker::visit(Connection* connection) {
	this->visit(dynamic_cast<Node*>(connection)); //just feed forward to be handled as node
}


void SubGraphChecker::reset(Id subGraphNodeId) {
	this->subGraphNodeId = subGraphNodeId;
	isInSubGraph = false;
	pathCount = 0;
}

}

}

/* EOF */
