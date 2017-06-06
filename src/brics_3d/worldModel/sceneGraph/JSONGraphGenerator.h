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

#ifndef RSG_JSONGRAPHGENERATOR_H_
#define RSG_JSONGRAPHGENERATOR_H_

#include "Node.h"
#include "Group.h"
#include "Connection.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "Connection.h"
#include "INodeVisitor.h"
#include "brics_3d/util/JSONTypecaster.h"

#include <sstream>

namespace brics_3d {
namespace rsg {

/**
 * @brief Node visitor that generates a RSG-JSON representation.
 * @ingroup sceneGraph
 */
class JSONGraphGenerator : public INodeVisitor {
public:
	JSONGraphGenerator();
	virtual ~JSONGraphGenerator();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);
	virtual void visit(Connection* connection);

	virtual void reset();

	std::string getJSON();
	libvariant::Variant& getJSONModel();
    libvariant::Variant getJSONById(Id id); //TODO reference?

protected:

	/// json model of graph
	libvariant::Variant model;

    /// Table that maps IDs to already generated JSON objects - so we can assemble them later
    std::map<Id, libvariant::Variant > jsonLookUpTable;

    /// Iterator for jsonLookUpTable
    std::map<Id, libvariant::Variant >::const_iterator jsonIterator;

	std::vector< Node* > alreadyVisitedNodes;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_JSONGRAPHGENERATOR_H_ */

/* EOF */
