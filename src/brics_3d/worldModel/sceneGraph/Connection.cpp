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

#include "Connection.h"

namespace brics_3d {

namespace rsg {

Connection::Connection() {

}

Connection::~Connection() {

}

void Connection::accept(INodeVisitor* visitor) {
	visitor->visit(this);
	if (visitor->getDirection() == INodeVisitor::upwards) {
		for(unsigned i = 0; i < getNumberOfParents(); ++i) // recursively go up the graph structure
		{
			getParent(i)->accept(visitor);
		}
	}
}

void Connection::addSourceNode(Node* node)
{
    sourceNodes.push_back(node);
}

void Connection::removeSourceNode(Node* node)
{

	std::vector<Node*>::iterator parentIterator = std::find(sourceNodes.begin(), sourceNodes.end(), node);
    if (parentIterator!=sourceNodes.end()) {
    	sourceNodes.erase(parentIterator);
    }
}

Node* Connection::getSourceNode(unsigned int index)  {
	if (index >= getNumberOfParents()) {
		assert(false);
		return 0;
	}
	return sourceNodes[index];
}

unsigned int Connection::getNumberOfSourceNodes() const {
	return static_cast<unsigned int>(sourceNodes.size());
}

void Connection::addTargetNode(Node* node)
{
    targetNodes.push_back(node);
}

void Connection::removeTargetNode(Node* node)
{

	std::vector<Node*>::iterator parentIterator = std::find(targetNodes.begin(), targetNodes.end(), node);
    if (parentIterator!=targetNodes.end()) {
    	targetNodes.erase(parentIterator);
    }
}

Node* Connection::getTargetNode(unsigned int index)  {
	if (index >= getNumberOfParents()) {
		assert(false);
		return 0;
	}
	return targetNodes[index];
}


unsigned int Connection::getNumberOfTargetNodes() const {
	return static_cast<unsigned int>(targetNodes.size());
}

} // namespace brics_3d::RSG

} // namespace brics_3d

/* EOF */

