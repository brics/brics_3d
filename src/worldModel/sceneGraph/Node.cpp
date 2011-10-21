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

vector<Attribute> Node::getAttributes() const
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


} // namespace BRICS_3D::RSG

} // namespace BRICS_3D

/* EOF */

