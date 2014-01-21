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

#ifndef RSG_DOTGRAPHGENERATOR_H_
#define RSG_DOTGRAPHGENERATOR_H_

#include "Node.h"
#include "Group.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "INodeVisitor.h"
#include "VisualizationConfiguration.h"

#include <sstream>

namespace brics_3d {

namespace rsg {

/**
 * @brief Node visitor that generates a "dot" representation.
 * @ingroup sceneGraph
 */
class DotGraphGenerator : public INodeVisitor {
public:
	DotGraphGenerator();
	virtual ~DotGraphGenerator();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);

	virtual void reset();

	std::string getDotGraph();

	const VisualizationConfiguration& getConfig() const {
		return config;
	}

	void setConfig(const VisualizationConfiguration& config) {
		this->config = config;
	}

protected:

	virtual void doHandleNode(Node* node); //could be overidden by more advanced/beautiful handlers
	virtual void doHandleTransform(Transform* node);
	virtual void doHandleGeometricNode(GeometricNode* node);
	virtual void doHandleEdges(Group* node);

	std::stringstream dotGraph;
	std::stringstream nodes;
	std::stringstream edges;

	std::vector< Node* > alreadyVisitedNodes;

	VisualizationConfiguration config;
};

}

}

#endif /* RSG_DOTGRAPHGENERATOR_H_ */

/* EOF */
