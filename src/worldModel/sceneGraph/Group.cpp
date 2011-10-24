/******************************************************************************
* Copyright (c) 2011
* GPS GmbH
*
* Author:
* Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of GPS GmbH nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include "Group.h"
#include "core/Logger.h"
#include <assert.h>

namespace BRICS_3D {

namespace RSG {

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

Group::NodePtr Group::getChild(unsigned int index) {
	return children[index];
}

unsigned int Group::getNumberOfChildren() const {
	return static_cast<unsigned int>(children.size());
}

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D

/* EOF */

