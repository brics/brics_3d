/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2017, KU Leuven
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

#ifndef NODEHASHTRAVERSER_H_
#define NODEHASHTRAVERSER_H_

#include "INodeVisitor.h"
#include "Group.h"
#include "Connection.h"
#include "Node.h"
#include "Transform.h"
#include "UncertainTransform.h"
#include "GeometricNode.h"
#include "Shape.h"
#include <map>

namespace brics_3d {
namespace rsg {

/**
 * @brief A traverser to generate a hash for a sub graph.
 *
 * The traverser traverses along the containment parent-child graph (DAG) in order to generate a Merkle DAG.
 *
 */
class NodeHashTraverser : public INodeVisitor {
public:
	NodeHashTraverser();
	virtual ~NodeHashTraverser();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);
	virtual void visit(Connection* connection);

	void reset();
	std::string getHashById(Id id);

	static const string NIL;

protected:
    /// Table that maps IDs to already generated hashes - so we do not have to recompute them.
    std::map<Id, std::string > hashLookUpTable;

    /// Iterator for idLookUpTable
    std::map<Id, std::string >::const_iterator hashIterator;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* NODEHASHTRAVERSER_H_ */

/* EOF */
