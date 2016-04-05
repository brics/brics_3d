/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2016, KU Leuven
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

#include "RootFinder.h"
#include "brics_3d/core/Logger.h"

namespace brics_3d {

namespace rsg {

RootFinder::RootFinder() : INodeVisitor(upwards) {
	reset();
}

RootFinder::~RootFinder() {

}

void RootFinder::visit(Node* node) {
	assert (node != 0);
	LOG(DEBUG) << "RootFinder: Visiting node with ID: " << node->getId();

	if (node->getNumberOfParents() == 0) { //root reached so memorize it
		rootNode = node->getId();
		LOG(DEBUG) << "\tRootFinder: Found root with ID: " << node->getId();
	}
}

void RootFinder::visit(Connection* connection) {
	this->visit(dynamic_cast<Node*>(connection)); //just feed forward to be handled as node
}

void RootFinder::visit(Group* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void RootFinder::visit(Transform* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void RootFinder::visit(GeometricNode* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void RootFinder::reset() {
	rootNode = 0;
}

}

}

/* EOF */
