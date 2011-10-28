/******************************************************************************
* Copyright (c) 2011
* GPS GmbH
*
* Author:
* Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of GPS GmbH nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include "SceneManager.h"
#include "SimpleIdGenerator.h"
#include "core/Logger.h"

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
	idLookUpTable.insert(std::make_pair(rootNode->getId(), rootNode.get()));
}

unsigned int SceneManager::getRootId() {
	return idGenerator->getRootId();
}

bool SceneManager::getNodeAttributes(unsigned int id, vector<Attribute>& attributes) {
	attributes.clear();
	Node* node = findNodeRecerence(id);
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
	Node* node = findNodeRecerence(id);
	if (node != 0) {
		for (unsigned int i = 0; i < node->getNumberOfParents(); ++i) {
			parentIds.push_back(node->getParent(i)->getId());
		}
		return true;
	}
	return false;
}

bool SceneManager::getGroupChildren(unsigned int id, vector<unsigned int>& childIds) {
	Node* node = findNodeRecerence(id);
	Group* group = dynamic_cast<Group*>(node);
	if (group != 0) {
		for (unsigned int i = 0; i < group->getNumberOfChildren(); ++i) {
			childIds.push_back(group->getChild(i)->getId());
		}
		return true;
	}
	LOG(ERROR) << "Node with ID " << id << " is not a group. Cannot return child IDs";
	return false;
}

bool SceneManager::addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes) {
	Node* node = findNodeRecerence(parentId);
	Group* parentGroup = dynamic_cast<Group*>(node);
	if (parentGroup != 0) {
		Node::NodePtr newNode(new Node());
		newNode->setId(idGenerator->getNextValidId());
		newNode->setAttributes(attributes);
		parentGroup->addChild(newNode);
		assignedId = newNode->getId();
		idLookUpTable.insert(std::make_pair(newNode->getId(), newNode.get()));
		return true;
	}
	LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new node as a child of it.";
	return false;
}

bool SceneManager::addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes) {
	Node* node = findNodeRecerence(parentId);
	Group* parentGroup = dynamic_cast<Group*>(node);
	if (parentGroup != 0) {
		Group::GroupPtr newGroup(new Group());
		newGroup->setId(idGenerator->getNextValidId());
		newGroup->setAttributes(attributes);
		parentGroup->addChild(newGroup);
		assignedId = newGroup->getId();
		idLookUpTable.insert(std::make_pair(newGroup->getId(), newGroup.get()));
		return true;
	}
	LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new group as a child of it.";
	return false;
}

bool SceneManager::addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
	Node* node = findNodeRecerence(parentId);
	Group* parentGroup = dynamic_cast<Group*>(node);
	if (parentGroup != 0) {
		RSG::Transform::TransformPtr newTransform(new Transform());
		newTransform->setId(idGenerator->getNextValidId());
		newTransform->setAttributes(attributes);
		newTransform->insertTransform(transform, timeStamp);
		parentGroup->addChild(newTransform);
		assignedId = newTransform->getId();
		idLookUpTable.insert(std::make_pair(newTransform->getId(), newTransform.get()));
		return true;
	}
	LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new transform as a child of it.";
	return false;
}

bool SceneManager::addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp) {
	Node* node = findNodeRecerence(parentId);
	Group* parentGroup = dynamic_cast<Group*>(node);
	if (parentGroup != 0) {
		GeometricNode::GeometricNodePtr newGeometricNode(new GeometricNode());
		newGeometricNode->setId(idGenerator->getNextValidId());
		newGeometricNode->setAttributes(attributes);
		newGeometricNode->setShape(shape);
		newGeometricNode->setTimeStamp(timeStamp);
		parentGroup->addChild(newGeometricNode);
		assignedId = newGeometricNode->getId();
		idLookUpTable.insert(std::make_pair(newGeometricNode->getId(), newGeometricNode.get()));
		return true;
	}
	LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new geometric node as a child of it.";
	return false;
}


bool SceneManager::setNodeAttributes(unsigned int id, vector<Attribute> newAttributes) {
	Node* node = findNodeRecerence(id);
	if (node != 0) {
		node->setAttributes(newAttributes);
		return true;
	}
	return false;
}

bool SceneManager::addParent(unsigned int id, unsigned int parentId) {
	Node* node = findNodeRecerence(parentId);
	Node* parentNode = findNodeRecerence(parentId);
	Group* parentGroup = dynamic_cast<Group*>(parentNode);
	if (parentGroup != 0 && node != 0) {
//		parentGroup->addChild(node);// TODO shared ref.
		return true;
	}
	LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new parent-child relation.";
	return false;
}


Node* SceneManager::findNodeRecerence(unsigned int id) {
	nodeIterator = idLookUpTable.find(id);
	if (nodeIterator != idLookUpTable.end()) { //TODO multiple IDs?
		assert (id == nodeIterator->second->getId()); // Otherwise something really went wrong while maintaining IDs...
		return nodeIterator->second;
	}
	LOG(WARNING) << "Scene graph does not contain a node with ID " << id;
	return 0;
}


} // namespace BRICS_3D::RSG

} // namespace BRICS_3D

/* EOF */

