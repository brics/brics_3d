/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2016, KU Leuven
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

#ifndef RSG_ROOTFINDER_H_
#define RSG_ROOTFINDER_H_

#include "INodeVisitor.h"
#include "Node.h"
#include "Group.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "Connection.h"

using std::vector;

namespace brics_3d {

namespace rsg {

/**
 * Graph traversal to find the root node.
 * @ingroup sceneGraph
 */
class RootFinder: public INodeVisitor {
public:
	RootFinder();
	virtual ~RootFinder();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);
	virtual void visit(Connection* connection);


	virtual void reset();

	const Id getRootNode() const {
		return rootNode;
	}

protected:
    Id rootNode;

};

}

}

#endif /* RSG_ROOTFINDER_H_ */

/* EOF */
