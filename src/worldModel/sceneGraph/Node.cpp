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

#include "Node.h"
#include "Attribute.h"
#include <stdexcept>

namespace BRICS_3D {

namespace RSG {

Node::Node() {
	this->id = 0;
	this->attributes.clear();
	this->parents.clear();
}

Node::~Node() {

}

vector<Attribute> Node::getAttributes()
{
    return attributes;
}

unsigned int Node::getId() const
{
    return id;
}

void Node::setAttributes(vector<Attribute> attributes)
{
    this->attributes = attributes;
}

void Node::setId(unsigned int id)
{
    this->id = id;
}

void Node::addParent(Node* node)
{
    parents.push_back(node);
}

void Node::removeParent(Node* node)
{

	std::vector<Node*>::iterator parentIterator = std::find(parents.begin(), parents.end(), node);
    if (parentIterator!=parents.end()) {
    	parents.erase(parentIterator);
    }
}

vector<Node*> Node::getParents() {
	return parents;
}

Node* Node::getParent(unsigned int index)  {
	if (index >= getNumberOfParents()) {
//		throw std::out_of_range("Cannot get parent for this Node."); //FIXME
		assert(false);
		return 0;
	}
	return parents[index];
}



unsigned int Node::getNumberOfParents() const {
	return static_cast<unsigned int>(parents.size());
}

void Node::accept(INodeVisitor* visitor) {
	visitor->visit(this);
	if (visitor->getDirection() == INodeVisitor::upwards) {
	    for(unsigned i = 0; i < getNumberOfParents(); ++i) // recursively go up the graph structure
	    {
	        getParent(i)->accept(visitor);
	    }
	}
}

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D

/* EOF */

