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

#include "PathCollector.h"

namespace brics_3d {

namespace rsg {

PathCollector::PathCollector() : INodeVisitor(upwards) {
	reset();
}

PathCollector::~PathCollector() {

}

void PathCollector::reset() {
	currentPath.clear();
	nodePaths.clear();
}

void PathCollector::visit(Node* node){
	assert (node != 0);
	currentPath.insert(currentPath.begin(), node);
	if (node->getNumberOfParents() == 0) { //root reached so collect data in new path
		if (static_cast<unsigned int>(nodePaths.size()) == 0u) { //remove the "caller's" pointer
				currentPath.pop_back();
		}
		nodePaths.push_back(currentPath);
		currentPath.clear();
	}
}

void PathCollector::visit(Group* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void PathCollector::visit(Transform* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void PathCollector::visit(GeometricNode* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

}

}

/* EOF */
