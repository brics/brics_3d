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

#ifndef PATHCOLLECTOR_H_
#define PATHCOLLECTOR_H_

#include "Node.h"
#include "Group.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "INodeVisitor.h"

namespace BRICS_3D {

namespace RSG {

/**
 * @brief Node visitor that collects paths.
 *
 * Computes the collected paths from a certain node (where the accept is called) to the root node.
 * The node from where the traversal is initiated will excluded from the paths.
 *
 */
class PathCollector : public INodeVisitor {
public:
	PathCollector();
	virtual ~PathCollector();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);

	virtual void reset();

	Node::NodePathList getNodePaths() const
    {
        return nodePaths;
    }


protected:

	Node::NodePath currentPath;
	Node::NodePathList nodePaths;

};

}

}

#endif /* PATHCOLLECTOR_H_ */

/* EOF */
