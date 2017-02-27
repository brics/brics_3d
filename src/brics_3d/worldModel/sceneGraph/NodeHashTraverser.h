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

namespace brics_3d {
namespace rsg {

class NodeHashTraverser : public INodeVisitor {
public:
	NodeHashTraverser();
	virtual ~NodeHashTraverser();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);
	virtual void visit(Connection* connection);


	// map
};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* NODEHASHTRAVERSER_H_ */

/* EOF */
