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

#ifndef GROUP_H
#define GROUP_H

#include "Node.h"
#include <vector>
using std::vector;

namespace BRICS_3D {

namespace RSG {

/**
 *  @brief The group allows for the <b>graph</b> relations, as it extends a node to have further children nodes.
 */
class Group : public Node {

public:

	typedef boost::shared_ptr<Group> GroupPtr;
	typedef boost::shared_ptr<Group const> GroupConstPtr;

    Group();

    virtual ~Group();
    virtual void addChild(NodePtr child);
    virtual void insertChild(NodePtr child, unsigned int index);
    void removeChild(NodePtr child);
    unsigned int getChildIndex(NodePtr node);
    virtual void removeChildren(unsigned int startIndex, unsigned int numberOfChildrenToRemove = 1);

    unsigned int getNumberOfChildren() const;

    NodePtr getChild(unsigned int index);

    virtual void accept(INodeVisitor* visitor);

  private:
    vector<NodePtr> children;

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

