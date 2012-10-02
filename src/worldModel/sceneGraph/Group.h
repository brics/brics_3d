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

#ifndef GROUP_H
#define GROUP_H

#include "Node.h"
#include <vector>
using std::vector;

namespace brics_3d {

namespace rsg {

/**
 *  @brief The group allows for the <b>graph</b> relations, as it extends a node to have further children nodes.
 *  @ingroup sceneGraph
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
    unsigned int getChildIndex(Node* node); //especially intresting for upward traversals
    virtual void removeChildren(unsigned int startIndex, unsigned int numberOfChildrenToRemove = 1);

    unsigned int getNumberOfChildren() const;

    NodePtr getChild(unsigned int index);

    virtual void accept(INodeVisitor* visitor);

  private:
    vector<NodePtr> children;

};

} // namespace brics_3d::RSG

} // namespace brics_3d
#endif

/* EOF */

