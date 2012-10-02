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

#include "Group.h"
#include "core/Logger.h"
#include <assert.h>

namespace brics_3d {

namespace rsg {

Group::Group() {
	this->children.clear();
}

Group::~Group() {

	/*
	 * clean up all "parent pointers" of all the children on deletion of this group,
	 * otherwise they will point to nirvana....
	 */
    for(unsigned i = 0; i < getNumberOfChildren(); ++i)
    {
        getChild(i)->removeParent(this);
    }
}

void Group::addChild(NodePtr child) {
	insertChild(child, children.size());
}

void Group::insertChild(NodePtr child, unsigned int index) {
	assert(child != 0);
	assert(child.get() != 0);

	/* update parent-child relation */
	if (index >= children.size()) {
		children.push_back(child);
	} else {
		children.insert(children.begin() + index, child);
	}

	/* update child-parent relation */
	child->addParent(this); // TODO does this create to many reference counts? => use <boost/enable_shared_from_this.hpp> ?
}

void Group::removeChild(NodePtr child) {
    unsigned int index = getChildIndex(child);
    if (index<children.size()) {
    	removeChildren(index);
    }
}

void Group::removeChildren(unsigned int startIndex, unsigned int numberOfChildrenToRemove) {
    unsigned int endOfRemoveRange = startIndex + numberOfChildrenToRemove;
    if (endOfRemoveRange > getNumberOfChildren()) // crop the range
    {
    	LOG(WARNING) << "Cropping the range of children to be removed.";
        endOfRemoveRange = getNumberOfChildren();
    }

    for(unsigned i= startIndex; i < endOfRemoveRange; ++i)
    {
        getChild(i)->removeParent(this);
    }

    children.erase(children.begin() + startIndex, children.begin() + endOfRemoveRange);
}

unsigned int Group::getChildIndex(NodePtr node){
    for (unsigned int childIndex = 0; childIndex < getNumberOfChildren(); ++childIndex)
    {
        if (children[childIndex] == node) {
        	return childIndex;
        }
    }
    return getNumberOfChildren(); // not found.
}

unsigned int Group::getChildIndex(Node* node) {
    for (unsigned int childIndex = 0; childIndex < getNumberOfChildren(); ++childIndex)
    {
        if (children[childIndex].get() == node) {
        	return childIndex;
        }
    }
    return getNumberOfChildren(); // not foun
}

Group::NodePtr Group::getChild(unsigned int index) {
	return children[index];
}

unsigned int Group::getNumberOfChildren() const {
	return static_cast<unsigned int>(children.size());
}

void Group::accept(INodeVisitor* visitor){
	visitor->visit(this);
	if (visitor->getDirection() == INodeVisitor::upwards) { //TODO move to "traverseUpwards" method?
	    for(unsigned i = 0; i < getNumberOfParents(); ++i) // recursively go up the graph structure
	    {
	        getParent(i)->accept(visitor);
	    }
	} else if (visitor->getDirection() == INodeVisitor::downwards) { //TODO move to "traverseDownwards" method?
		for(unsigned i = 0; i < getNumberOfChildren(); ++i) // recursively go down the graph structure
		{
			getChild(i)->accept(visitor);
		}
	}
}

} // namespace brics_3d::RSG

} // namespace brics_3d

/* EOF */

