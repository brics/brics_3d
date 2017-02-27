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

#include "NodeHashTraverser.h"

namespace brics_3d {
namespace rsg {

NodeHashTraverser::NodeHashTraverser() : INodeVisitor(custom) {

}

NodeHashTraverser::~NodeHashTraverser() {

}

void NodeHashTraverser::visit(Node* node) {
}

void NodeHashTraverser::visit(Group* node) {
}

void NodeHashTraverser::visit(Transform* node) {
}

void NodeHashTraverser::visit(GeometricNode* node) {
}

void NodeHashTraverser::visit(Connection* connection) {
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
