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

#include "GeometricNode.h"

namespace BRICS_3D {

namespace RSG {

GeometricNode::GeometricNode() {
  // Bouml preserved body begin 0002B383
  // Bouml preserved body end 0002B383
}

GeometricNode::~GeometricNode() {
  // Bouml preserved body begin 0002B403
  // Bouml preserved body end 0002B403
}


void GeometricNode::accept(INodeVisitor* visitor) {
	visitor->visit(this);
	if (visitor->getDirection() == INodeVisitor::upwards) {
		for(unsigned i = 0; i < getNumberOfParents(); ++i) // recursively go up the graph structure
		{
			getParent(i)->accept(visitor);
		}
	}
}

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D

/* EOF */

