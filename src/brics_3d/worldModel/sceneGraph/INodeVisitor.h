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

#ifndef RSG_INODEVISITOR_H_
#define RSG_INODEVISITOR_H_

namespace brics_3d {

namespace rsg {

class Node;
class Group;
class Transform;
class GeometricNode;
class Connection;

/**
 * @brief Abstract interface for visitors of the scene graph (Visitor pattern).
 * @ingroup sceneGraph
 */
class INodeVisitor {
public:

	enum TraverseDirection {
		downwards, // up
		upwards,   // down
		custom     // specialized traversal e.b. with memorization of intermediate results
	};

	INodeVisitor(TraverseDirection direction = downwards){this->direction = direction;};
	virtual ~INodeVisitor(){};

	virtual void visit(Node* node){};
	virtual void visit(Group* node){};
	virtual void visit(Transform* node){};
	virtual void visit(GeometricNode* node){};
	virtual void visit(Connection* connection) = 0;//{};

    TraverseDirection getDirection() const
    {
        return direction;
    }

    void setDirection(TraverseDirection direction)
    {
        this->direction = direction;
    }

protected:
	TraverseDirection direction;

};

}

}

#endif /* RSG_INODEVISITOR_H_ */

/* EOF */
