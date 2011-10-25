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

#ifndef NODE_H
#define NODE_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include "Attribute.h"
#include "INodeVisitor.h"

using std::vector;

namespace BRICS_3D {

namespace RSG {

class Group;



/**
 *  @brief A node in the robot scenegraph.
 */
class Node {

public:

	typedef boost::shared_ptr<Node> NodePtr;
	typedef boost::shared_ptr<Node const> NodeConstPtr;

	Node();

	virtual ~Node();

	vector<Attribute> getAttributes() const;
	unsigned int getId() const;

	void setAttributes(vector<Attribute> attributes);
	void setId(unsigned int id);


	vector<Node*> getParents(); //TODO dangerous as it reveals the pointers?!?

	Node* getParent(unsigned int index);

    unsigned int getNumberOfParents() const;

    virtual void accept(INodeVisitor* visitor);


private:

	void addParent(Node* node);
	void removeParent(Node* node);
	friend class BRICS_3D::RSG::Group; //only this one will be allowed to add parent-child relations

	/// Unique ID that will help to identify a certain node.
	unsigned int id;

	/// List of attributes for each node. Can be used to attach (semantic) tags.
	vector<Attribute> attributes;

	/// List of pointers to the parent Nodes.
	vector<Node*> parents; //these are rather weak references to prevent cyclic strong pointers

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

