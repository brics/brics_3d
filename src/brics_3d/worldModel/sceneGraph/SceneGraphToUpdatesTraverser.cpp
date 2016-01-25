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

#include "SceneGraphToUpdatesTraverser.h"
#include "brics_3d/core/Logger.h"

using brics_3d::Logger;

namespace brics_3d {

namespace rsg {

SceneGraphToUpdatesTraverser::SceneGraphToUpdatesTraverser(ISceneGraphUpdate* updatesRecieverHandle) {
	this->updatesRecieverHandle = updatesRecieverHandle;
	assert(updatesRecieverHandle != 0);
	enableForcedIds = true; // default value
	reset();
}

SceneGraphToUpdatesTraverser::~SceneGraphToUpdatesTraverser() {

}

void SceneGraphToUpdatesTraverser::visit(Node* node) {
	Id nodeId = node->getId();
	Id parentId;
	if(!handleExistingNode(node, parentId)) {
		updatesRecieverHandle->addNode(parentId, nodeId, node->getAttributes(), enableForcedIds);
	}
}

void SceneGraphToUpdatesTraverser::visit(Group* node) {
	Id nodeId = node->getId();
	Id parentId;
	if(!handleExistingNode(node, parentId)) {
		updatesRecieverHandle->addGroup(parentId, nodeId, node->getAttributes(), enableForcedIds);
//		alreadyVisitedNodes.push_back(node);
	}
}

void SceneGraphToUpdatesTraverser::visit(Transform* node) {
	Id nodeId = node->getId();
	Id parentId;
	if(!handleExistingNode(node, parentId)) {
		updatesRecieverHandle->addTransformNode(parentId, nodeId, node->getAttributes(), node->getLatestTransform(), node->getLatestTimeStamp(), enableForcedIds);
	}
}

void SceneGraphToUpdatesTraverser::visit(GeometricNode* node) {
	Id nodeId = node->getId();
	Id parentId;
	if(!handleExistingNode(node, parentId)) {
		updatesRecieverHandle->addGeometricNode(parentId, nodeId, node->getAttributes(), node->getShape(), node->getTimeStamp(), enableForcedIds);
	}
}

void SceneGraphToUpdatesTraverser::visit(Connection* connection) {
	Id connectionId = connection->getId();
	Id parentId;
	if(!handleExistingNode(connection, parentId)) {
		vector<Id> sourceIds;
		vector<Id> targetIds;
		for (unsigned int i = 0; i < connection->getNumberOfSourceNodes(); ++i) {
			sourceIds.push_back(connection->getSourceNode(i)->getId());
		}
		for (unsigned int i = 0; i < connection->getNumberOfTargetNodes(); ++i) {
			targetIds.push_back(connection->getTargetNode(i)->getId());
		}

		updatesRecieverHandle->addConnection(parentId,
				connectionId,
				connection->getAttributes(),
				sourceIds,
				targetIds,
				connection->getStart(),
				connection->getEnd(),
				enableForcedIds);
	}

}



void SceneGraphToUpdatesTraverser::reset() {
	alreadyVisitedNodesWithPendingStatus.clear();
}

bool SceneGraphToUpdatesTraverser::getEnableForcedIds() {
    return enableForcedIds;
}

void SceneGraphToUpdatesTraverser::setEnableForcedIds(bool enableForcedIds) {
    this->enableForcedIds = enableForcedIds;
}


bool SceneGraphToUpdatesTraverser::handleExistingNode(Node* node, Id& parentId) {
	Id nodeId = node->getId();
	vector<Node*> parents;


//	if(alreadyVisitedNodesWithPendingStatus.size() == 0) { //first element of traversal shall be ignored
//		LOG(DEBUG) << "SceneGraphToUpdatesTraverser: Ignoring root node of traversal.";
//		parents.clear();
//		alreadyVisitedNodesWithPendingStatus.insert(std::make_pair(node, parents));
//		return true;
//	}


	parents = node->getParents();
	if (parents.size() == 0) { // THE root node
		LOG(DEBUG) << "SceneGraphToUpdatesTraverser: Found a root node as it has no parents. Triggering addRemoteRootNode.";
		updatesRecieverHandle->addRemoteRootNode(node->getId(), node->getAttributes());
		parents.clear();
		alreadyVisitedNodesWithPendingStatus.insert(std::make_pair(node, parents));
		return true;
	}
	Node* potentialParentNode;
	assert(parents.size() > 0);
	potentialParentNode = parents[0];


	bool nodeWasAlreadyVisited = false;
	alreadyVisitedNodesIterator = alreadyVisitedNodesWithPendingStatus.find(node);
	if (alreadyVisitedNodesIterator != alreadyVisitedNodesWithPendingStatus.end()) {
		nodeWasAlreadyVisited = true;
	}

	if(nodeWasAlreadyVisited) { // so we add a new parent-child relation to the existin node

		/*
		 * loop over all pending parent IDs and check if it is possible to add parents.
		 * If yes remove from pending list
		 */
		for (unsigned int i = 0; i < alreadyVisitedNodesIterator->second.size(); ++i) {
			Node* parentCandidate = alreadyVisitedNodesIterator->second[i];
			alreadyVisitedAndPendingNodesIterator = alreadyVisitedNodesWithPendingStatus.find(parentCandidate);
			if (alreadyVisitedAndPendingNodesIterator != alreadyVisitedNodesWithPendingStatus.end()) { //for each?
				updatesRecieverHandle->addParent(nodeId, parentCandidate->getId());
				alreadyVisitedNodesIterator->second.erase(alreadyVisitedNodesIterator->second.begin() + i);
			}
		}

		return true;

	} else { // here we need to _create_ the node - which is done outsied the scope of this function, but the caller will be informed be the retured "false"
		parentId = potentialParentNode->getId();
		parents.erase(parents.begin());
		alreadyVisitedNodesWithPendingStatus.insert(std::make_pair(node, parents));
		return false;
	}

}

}

}

/* EOF */
