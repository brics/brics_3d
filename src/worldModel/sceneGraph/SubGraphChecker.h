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

#ifndef SUBGRAPHCHECKER_H_
#define SUBGRAPHCHECKER_H_

#include "INodeVisitor.h"
#include "Node.h"
#include "Group.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "Attribute.h"

namespace BRICS_3D {

namespace RSG {

/**
 * @brief A traverser to check if a certain node is in a subgaph of another node.
 */
class SubGraphChecker: public INodeVisitor {
public:

	/**
	 * @brief Contuctor.
	 * @param subGraphNodeId Same as in BRICS_3D::RSG::SubGraphChecker::subGraphNodeId
	 */
	SubGraphChecker(unsigned int subGraphNodeId);

	/**
	 * @brief Default constructor.
	 * @return
	 */
	virtual ~SubGraphChecker();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);

	/**
	 * @brief Resets the query.
	 * @param subGraphNodeId Same as in BRICS_3D::RSG::SubGraphChecker::subGraphNodeId
	 */
	virtual void reset(unsigned int subGraphNodeId);

    unsigned int getPathCount()
    {
        return pathCount;
    }

    bool nodeIsInSubGraph()
    {
        return isInSubGraph;
    }

protected:

	/// Specifies the "root" node of the subgraph of intrest.
	unsigned int subGraphNodeId;

	/**
	 * Result of traversal. Will be true if the node where the traveral was started is
	 * in the subgraph of the node specified by subGraphNodeId.
	 */
	bool isInSubGraph;

	/// Number of possible paths from start node to subGraphNodeId. Note: pathCount >= 1 is equivalent to isInSubGraph == true.
	unsigned int pathCount;
};

}

}

#endif /* SUBGRAPHCHECKER_H_ */

/* EOF */
