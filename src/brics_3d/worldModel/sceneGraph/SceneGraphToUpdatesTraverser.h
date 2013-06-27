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

#ifndef RSG_SCENEGRAPHTOUPDATESTRAVERSER_H_
#define RSG_SCENEGRAPHTOUPDATESTRAVERSER_H_

#include "Node.h"
#include "Group.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "INodeVisitor.h"
#include "ISceneGraphUpdate.h"

#include <map>

namespace brics_3d {

namespace rsg {

/**
 * @brief Scene graph visitor that traverses an existing graph and will call approproate update functions for each element.
 *
 * This traverser can be used i.e. to duplicate an existing scene graph.
 *
 * There are a couple of implementational issues to keep in mind:
 * - To be able to successfully add new elements in the observing/receiving graph only already existing nodes
 * can be referenced, in particular via the parent IDs. Only nodes that have been traversed before can be
 * assumed to exist already.
 * - During a traversal the visited elements do not show which preceeding parent caused the invokation.
 * This needs to be recovered.
 * - Potential duplications of parent-child relation could occour while traversing a more complex graph.
 *
 * @ingroup sceneGraph
 */
class SceneGraphToUpdatesTraverser : public INodeVisitor {
public:

	/**
	 * @brief Constructor.
	 * @param updatesRecieverHandle Pointer to the reciever of all updates.
	 */
	SceneGraphToUpdatesTraverser(ISceneGraphUpdate* updatesRecieverHandle);

	/**
	 * @brief Default destructor.
	 */
	virtual ~SceneGraphToUpdatesTraverser();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);

	/// Reset all memory.
	virtual void reset();

    bool getEnableForcedIds();
    void setEnableForcedIds(bool enableForcedIds);

private:

	/**
	 * @brief The common part for all nodes. Only the creation of new nodes differs.
	 * @param node The current node to be processed.
	 * @param[out] parentId The deduced parent ID such that a new node can be created in case false will be returned.
	 * @return True if node exists and was sucessfully handeled or false if node could not be handled
	 * and needs to be treated as a new node.
	 */
	bool handleExistingNode(Node* node, unsigned int& parentId);

	/**
	 * This is the reciever of all updates.
	 * Could be an inmlemation an an observer according to ISceneGraphUpdateObserver or another SceneGrapgFacade.
	 */
	ISceneGraphUpdate* updatesRecieverHandle;

	/// This will be passed as flag to the update functions. Default is true.
	bool enableForcedIds;

	/// Memory of what already visited nodes, including wich partent per node have been already added.
	std::map<Node*, vector<Node*> > alreadyVisitedNodesWithPendingStatus;
	std::map<Node*, vector<Node*> >::iterator alreadyVisitedNodesIterator;
	std::map<Node*, vector<Node*> >::iterator alreadyVisitedAndPendingNodesIterator;

};

}

}

#endif /* RSG_SCENEGRAPHTOUPDATESTRAVERSER_H_ */

/* EOF */
