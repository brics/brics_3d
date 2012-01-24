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

#include "SceneManager.h"
#include "SimpleIdGenerator.h"
#include "core/Logger.h"
#include "AttributeFinder.h"

namespace BRICS_3D {

namespace RSG {

void SceneManager::findSceneNodes(const Attribute & attributes, Node & nodeReferences) {
  // Bouml preserved body begin 00022703
  // Bouml preserved body end 00022703
}



SceneManager::SceneManager() {
	this->idGenerator = new SimpleIdGenerator();
	initialize();
}

SceneManager::SceneManager(IIdGenerator* idGenerator) {
	assert(idGenerator != 0);
	this->idGenerator = idGenerator;
	initialize();
}

SceneManager::~SceneManager() {
	if (idGenerator) {
		delete idGenerator;
		idGenerator = 0;
	}
}

void SceneManager::initialize() {
	rootNode = Group::GroupPtr(new Group());
	rootNode->setId(idGenerator->getRootId());
	assert(rootNode->getId() == idGenerator->getRootId());
	idLookUpTable.insert(std::make_pair(rootNode->getId(), rootNode));
}

unsigned int SceneManager::getRootId() {
	return idGenerator->getRootId();
}

bool SceneManager::getNodes(vector<Attribute> attributes, vector<unsigned int>& ids) {
	ids.clear();
	Node::NodeWeakPtr tmpNode = findNodeRecerence(getRootId());
	Node::NodePtr node = tmpNode.lock();
	if (node != 0) {
			AttributeFinder attributeFinder;
			attributeFinder.setQueryAttributes(attributes);
			node->accept(&attributeFinder);
			for (unsigned int i = 0; i < static_cast<unsigned int>(attributeFinder.getMatchingNodes().size()) ; ++i) {
				ids.push_back((*attributeFinder.getMatchingNodes()[i]).getId());
			}

		return true;
	}
	LOG(ERROR) << "Cannot find root node. Aborting attribute search.";
	return false;
}

bool SceneManager::getNodeAttributes(unsigned int id, vector<Attribute>& attributes) {
	attributes.clear();
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	if (node != 0) {
		if (node != 0) {
			for (unsigned int i = 0; i < static_cast<unsigned int>(node->getAttributes().size()) ; ++i) {
				attributes.push_back(node->getAttributes()[i]);
			}
		}
		return true;
	}
	return false;
}

bool SceneManager::getNodeParents(unsigned int id, vector<unsigned int>& parentIds) {
	parentIds.clear();
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	if (node != 0) {
		for (unsigned int i = 0; i < node->getNumberOfParents(); ++i) {
			parentIds.push_back(node->getParent(i)->getId());
		}
		return true;
	}
	return false;
}

bool SceneManager::getGroupChildren(unsigned int id, vector<unsigned int>& childIds) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	Group::GroupPtr group = boost::dynamic_pointer_cast<Group>(node);
	if (group != 0) {
		for (unsigned int i = 0; i < group->getNumberOfChildren(); ++i) {
			childIds.push_back(group->getChild(i)->getId());
		}
		return true;
	}
	LOG(ERROR) << "Node with ID " << id << " is not a group. Cannot return child IDs";
	return false;
}

bool SceneManager::getTransform(unsigned int id, TimeStamp timeStamp, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	RSG::Transform::TransformPtr transformNode = boost::dynamic_pointer_cast<RSG::Transform>(node);
	if (transformNode != 0) {
		transform = transformNode->getTransform(timeStamp);
		return true;
	}
	LOG(ERROR) << "Node with ID " << id << " is not a transform. Cannot return transform data.";
	return false;
}

bool SceneManager::getGeometry(unsigned int id, Shape::ShapePtr& shape, TimeStamp& timeStamp) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	GeometricNode::GeometricNodePtr geometricNode = boost::dynamic_pointer_cast<GeometricNode>(node);
	if (geometricNode != 0) {
		shape = geometricNode->getShape();
		timeStamp = geometricNode->getTimeStamp();
		return true;
	}
	LOG(ERROR) << "Node with ID " << id << " is not a geometric node. Cannot return shape data.";
	return false;
}

bool SceneManager::addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(parentId);
	Node::NodePtr node = tmpNode.lock();
	Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(node);
	if (parentGroup != 0) {
		Node::NodePtr newNode(new Node());
		newNode->setId(idGenerator->getNextValidId());
		newNode->setAttributes(attributes);
		parentGroup->addChild(newNode);
		assignedId = newNode->getId();
		idLookUpTable.insert(std::make_pair(newNode->getId(), newNode));
		return true;
	}
	LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new node as a child of it.";
	return false;
}

bool SceneManager::addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(parentId);
	Node::NodePtr node = tmpNode.lock();
	Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(node);
	if (parentGroup != 0) {
		Group::GroupPtr newGroup(new Group());
		newGroup->setId(idGenerator->getNextValidId());
		newGroup->setAttributes(attributes);
		parentGroup->addChild(newGroup);
		assignedId = newGroup->getId();
		idLookUpTable.insert(std::make_pair(newGroup->getId(), newGroup));
		return true;
	}
	LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new group as a child of it.";
	return false;
}

bool SceneManager::addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(parentId);
	Node::NodePtr node = tmpNode.lock();
	//	Group* parentGroup = dynamic_cast<Group*>(node);
		Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(node);
	if (parentGroup != 0) {
		RSG::Transform::TransformPtr newTransform(new Transform());
		newTransform->setId(idGenerator->getNextValidId());
		newTransform->setAttributes(attributes);
		newTransform->insertTransform(transform, timeStamp);
		parentGroup->addChild(newTransform);
		assignedId = newTransform->getId();
		idLookUpTable.insert(std::make_pair(newTransform->getId(), newTransform));
		return true;
	}
	LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new transform as a child of it.";
	return false;
}

bool SceneManager::addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(parentId);
	Node::NodePtr node = tmpNode.lock();
	//	Group* parentGroup = dynamic_cast<Group*>(node);
	Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(node);
	if (parentGroup != 0) {
		GeometricNode::GeometricNodePtr newGeometricNode(new GeometricNode());
		newGeometricNode->setId(idGenerator->getNextValidId());
		newGeometricNode->setAttributes(attributes);
		newGeometricNode->setShape(shape);
		newGeometricNode->setTimeStamp(timeStamp);
		parentGroup->addChild(newGeometricNode);
		assignedId = newGeometricNode->getId();
		idLookUpTable.insert(std::make_pair(newGeometricNode->getId(), newGeometricNode));
		return true;
	}
	LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new geometric node as a child of it.";
	return false;
}


bool SceneManager::setNodeAttributes(unsigned int id, vector<Attribute> newAttributes) {
	//	Node* node = findNodeRecerence(id);
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	if (node != 0) {
		node->setAttributes(newAttributes);
		return true;
	}
	return false;
}

bool SceneManager::setTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	RSG::Transform::TransformPtr transformNode = boost::dynamic_pointer_cast<RSG::Transform>(node);
	if (transformNode != 0) {
		transformNode->insertTransform(transform, timeStamp);
		return true;
	}
	LOG(ERROR) << "Node with ID " << id << " is not a transform. Cannot set new transform data.";
	return false;
}

bool SceneManager::deleteNode(unsigned int id) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	if (node != 0) {
		if (node->getNumberOfParents() == 0) { // oops we are trying to delete the root node, but this on has no parents...
			assert (id == getRootId()); // just to be sure something really strange did not happend...
			LOG(WARNING) << "The root node with ID " << id << " cannot be deleted.";
			return false;
		}

		/*
		 * so we found the handle to the current node; now we will invoke
		 * the according delete function for every parent
		 */
		while (node->getNumberOfParents() > 0) { //NOTE: node->getNumberOfParents() will decrease within every iteration...
			unsigned int i = 0;
			RSG::Node* parentNode;
			parentNode = node->getParent(i);
			Group* parentGroup =  dynamic_cast<Group*>(parentNode);
			if (parentGroup != 0 ) {
				parentGroup->removeChild(node);
			} else {
				assert(false); // actually parents need to be groups otherwise sth. really went wrong
			}
		}
		idLookUpTable.erase(id); //erase by ID (if not done here there would be orphaned IDs)
		return true;
	}
	return false;
}

bool SceneManager::addParent(unsigned int id, unsigned int parentId) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	Node::NodeWeakPtr tmpParentNode = findNodeRecerence(parentId);
	Node::NodePtr parentNode = tmpParentNode.lock();
	Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(parentNode);

	if (parentGroup != 0 && node != 0) {
		parentGroup->addChild(node);// TODO shared ref.
		return true;
	}
	LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new parent-child relation.";
	return false;
}


Node::NodeWeakPtr SceneManager::findNodeRecerence(unsigned int id) {
	nodeIterator = idLookUpTable.find(id);
	if (nodeIterator != idLookUpTable.end()) { //TODO multiple IDs?
		Node::NodePtr tmpNodeHandle = nodeIterator->second.lock();
		if(tmpNodeHandle !=0) {
			assert (id == tmpNodeHandle->getId()); // Otherwise something really went wrong while maintaining IDs...
			return nodeIterator->second;
		}
		LOG(WARNING) << "ID " << id << " seems to be orphaned. Possibly its node has been deleted earlier.";
	}
	LOG(WARNING) << "Scene graph does not contain a node with ID " << id;
	return Node::NodeWeakPtr(); // should be kind of null...
}

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D

/* EOF */

