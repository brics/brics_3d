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

#ifndef RSG_SUBGRAPHCHECKER_H_
#define RSG_SUBGRAPHCHECKER_H_

#include "INodeVisitor.h"
#include "Node.h"
#include "Group.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "Connection.h"
#include "Attribute.h"

namespace brics_3d {

namespace rsg {

/**
 * @brief A traverser to check if a certain node is in a subgaph of another node.
 */
class SubGraphChecker: public INodeVisitor {
public:

	/**
	 * @brief Contuctor.
	 * @param subGraphNodeId Same as in brics_3d::rsg::SubGraphChecker::subGraphNodeId
	 */
	SubGraphChecker(Id subGraphNodeId);

	/**
	 * @brief Default constructor.
	 * @return
	 */
	virtual ~SubGraphChecker();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);
	virtual void visit(Connection* connection);


	/**
	 * @brief Resets the query.
	 * @param subGraphNodeId Same as in brics_3d::rsg::SubGraphChecker::subGraphNodeId
	 */
	virtual void reset(Id subGraphNodeId);

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
	Id subGraphNodeId;

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

#endif /* RSG_SUBGRAPHCHECKER_H_ */

/* EOF */
