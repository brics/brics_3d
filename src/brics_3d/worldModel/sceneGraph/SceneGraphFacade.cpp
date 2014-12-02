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

#include "SceneGraphFacade.h"
#include "SimpleIdGenerator.h"
#include "UuidGenerator.h"
#include "brics_3d/core/Logger.h"
#include "AttributeFinder.h"

#include <iomanip> // setprecision

namespace brics_3d {

namespace rsg {

SceneGraphFacade::SceneGraphFacade() {
//	this->idGenerator = new SimpleIdGenerator();
	callObserversEvenIfErrorsOccurred = true;
	this->idGenerator = new UuidGenerator();
	initialize();
}

SceneGraphFacade::SceneGraphFacade(IIdGenerator* idGenerator) {
	assert(idGenerator != 0);
	this->idGenerator = idGenerator;
	initialize();
}

SceneGraphFacade::~SceneGraphFacade() {
	if (idGenerator) {
		delete idGenerator;
		idGenerator = 0;
	}
}

void SceneGraphFacade::initialize() {
	rootNode = Group::GroupPtr(new Group());
	rootNode->setId(idGenerator->getRootId());
	assert(rootNode->getId() == idGenerator->getRootId());
	idLookUpTable.insert(std::make_pair(rootNode->getId(), rootNode));
	updateObservers.clear();
}

Id SceneGraphFacade::getRootId() {
	if(idGenerator) {
		return idGenerator->getRootId();
	} else {
		LOG(ERROR) << "Cannot return the root node as idGenerator is null. Possibly some else called the destructor. Try to reinitialize.";
		Id nil = 0;
		return nil;
	}
}

bool SceneGraphFacade::addRemoteRootNode(Id rootId, vector<Attribute> attributes) {
	bool operationSucceeded = false;
	bool idIsOk = false;

	if( (!doesIdExist(rootId)) && (idGenerator->removeIdFromPool(rootId)) ) {
		idIsOk = true;
	} else {
		LOG(WARNING) << "Remote root ID " << rootId << " cannot be assigend. Probably another object with that ID exists already!";
		idIsOk = false;
	}


	if ((idIsOk)) {
		Group::GroupPtr newGroup(new Group());
		newGroup->setId(rootId);
		newGroup->setAttributes(attributes);
		//parentGroup->addChild(newNode);
		remoteRootNodes.push_back(newGroup);
		idLookUpTable.insert(std::make_pair(newGroup->getId(), newGroup));
		operationSucceeded = true;
	}

//	/* Call all observers depending on the given policy in case an error occured */
//	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
//	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
//		(*observerIterator)->addRemoteRootNode(rootId, attributes);
//	}

	return operationSucceeded;
}

bool SceneGraphFacade::getNodes(vector<Attribute> attributes, vector<Id>& ids) {
	LOG(DEBUG) << " Current idLookUpTable lenght = " << idLookUpTable.size();
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

bool SceneGraphFacade::getNodeAttributes(Id id, vector<Attribute>& attributes) {
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

bool SceneGraphFacade::getNodeParents(Id id, vector<Id>& parentIds) {
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

bool SceneGraphFacade::getGroupChildren(Id id, vector<Id>& childIds) {
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

bool SceneGraphFacade::getTransform(Id id, TimeStamp timeStamp, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	rsg::Transform::TransformPtr transformNode = boost::dynamic_pointer_cast<rsg::Transform>(node);
	if (transformNode != 0) {
		transform = transformNode->getTransform(timeStamp);
		return true;
	}
	LOG(ERROR) << "Node with ID " << id << " is not a transform. Cannot return transform data.";
	return false;
}

bool SceneGraphFacade::getUncertainTransform(Id id, TimeStamp timeStamp, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform, ITransformUncertainty::ITransformUncertaintyPtr &uncertainty) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	rsg::UncertainTransform::UncertainTransformPtr transformNode = boost::dynamic_pointer_cast<rsg::UncertainTransform>(node);
	if (transformNode != 0) {
		transform = transformNode->getTransform(timeStamp);
		uncertainty = transformNode->getTransformUncertainty(timeStamp);
		return true;
	}
	LOG(ERROR) << "Node with ID " << id << " is not an uncertain transform. Cannot return transform data.";
	return false;
}

bool SceneGraphFacade::getGeometry(Id id, Shape::ShapePtr& shape, TimeStamp& timeStamp) {
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

bool SceneGraphFacade::getTransformForNode (Id id, Id idReferenceNode, TimeStamp timeStamp, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	Node::NodeWeakPtr tmpReferenceNode = findNodeRecerence(idReferenceNode);
	Node::NodePtr referenceNode= tmpReferenceNode.lock();
	if ((node != 0) && (referenceNode != 0)) {
//		transform = getGlobalTransform(node);
		transform = getTransformBetweenNodes(node, referenceNode, timeStamp);
		return true;
	}
	return false;
}


bool SceneGraphFacade::addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId) {
	bool operationSucceeded = false;
	bool idIsOk = false;
	Id id;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(parentId);
	Node::NodePtr node = tmpNode.lock();
	Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(node);

	if (forcedId && (parentGroup != 0)) { // NOTE: As we are handling existing IDs we need to be careful with removing them from the pool
		if( (!doesIdExist(assignedId)) && (idGenerator->removeIdFromPool(assignedId)) ) {
			id = assignedId;
			idIsOk = true;
		} else {
			LOG(WARNING) << "Forced ID " << assignedId << " cannot be assigend. Probably another object with that ID exists already!";
			idIsOk = false;
		}
	} else {
		id = idGenerator->getNextValidId();
		idIsOk = true;
	}

	if ((parentGroup != 0) && (idIsOk)) {
		Node::NodePtr newNode(new Node());
		newNode->setId(id);
		newNode->setAttributes(attributes);
		parentGroup->addChild(newNode);
		assignedId = newNode->getId();
		idLookUpTable.insert(std::make_pair(newNode->getId(), newNode));
		operationSucceeded = true;
	}

	/* Call all observers depending on the given policy in case an error occured */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		Id assignedIdcopy = assignedId; // prevent that observer might change this....
		(*observerIterator)->addNode(parentId, assignedIdcopy, attributes);
	}

	if (operationSucceeded) {
		return true;
	} else {
		if (idIsOk) { //If ID was OK but operation did not succeeded, then it is because of the parent...
			LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new node as a child of it.";
		}
		return false;
	}
}

bool SceneGraphFacade::addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId) {
	bool operationSucceeded = false;
	bool idIsOk = false;
	Id id;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(parentId);
	Node::NodePtr node = tmpNode.lock();
	Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(node);

	if (forcedId && (parentGroup != 0)) { // NOTE: As we are handling existing IDs we need to be careful with removing them from the pool
		if( (!doesIdExist(assignedId)) && (idGenerator->removeIdFromPool(assignedId)) ) {
			id = assignedId;
			idIsOk = true;
		} else {
			LOG(WARNING) << "Forced ID " << assignedId << " cannot be assigend. Probably another object with that ID exists already!";
			idIsOk = false;
		}
	} else {
		id = idGenerator->getNextValidId();
		idIsOk = true;
	}


	if ((parentGroup != 0) && (idIsOk)) {
		Group::GroupPtr newGroup(new Group());
		newGroup->setId(id);
		newGroup->setAttributes(attributes);
		parentGroup->addChild(newGroup);
		assignedId = newGroup->getId();
		idLookUpTable.insert(std::make_pair(newGroup->getId(), newGroup));
		operationSucceeded = true;
	}

	/* Call all observers depending on the given policy in case an error occured */
	if (operationSucceeded || callObserversEvenIfErrorsOccurred) {
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			Id assignedIdcopy = assignedId; // prevent that observer might change this....
			(*observerIterator)->addGroup(parentId, assignedIdcopy, attributes, forcedId);
		}
	}

	if (operationSucceeded) {
		return true;
	} else {
		if (idIsOk) { //If ID was OK but operation did not succeeded, then it is because of the parent...
			LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new group as a child of it.";
		}
		return false;
	}
}

bool SceneGraphFacade::addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId) {
	bool operationSucceeded = false;
	bool idIsOk = false;
	Id id;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(parentId);
	Node::NodePtr node = tmpNode.lock();

	Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(node);

	if (forcedId && (parentGroup != 0)) { // NOTE: As we are handling existing IDs we need to be careful with removing them from the pool
		if( (!doesIdExist(assignedId)) && (idGenerator->removeIdFromPool(assignedId)) ) {
			id = assignedId;
			idIsOk = true;
		} else {
			LOG(WARNING) << "Forced ID " << assignedId << " cannot be assigend. Probably another object with that ID exists already!";
			idIsOk = false;
		}
	} else {
		id = idGenerator->getNextValidId();
		idIsOk = true;
	}

	if ((parentGroup != 0) && (idIsOk)) {
		rsg::Transform::TransformPtr newTransform(new Transform());
		newTransform->setId(id);
		newTransform->setAttributes(attributes);
		newTransform->insertTransform(transform, timeStamp);
		parentGroup->addChild(newTransform);
		assignedId = newTransform->getId();
		idLookUpTable.insert(std::make_pair(newTransform->getId(), newTransform));
		operationSucceeded = true;
	}

	/* Call all observers depending on the given policy in case an error occured */
	if (operationSucceeded || callObserversEvenIfErrorsOccurred) {
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			Id assignedIdcopy = assignedId; // prevent that observer might change this....
			(*observerIterator)->addTransformNode(parentId, assignedIdcopy, attributes, transform, timeStamp);
		}
	}

	if(operationSucceeded) {
		return true;
	} else {
		if (idIsOk) { //If ID was OK but operation did not succeeded, then it is because of the parent...
			LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new transform as a child of it.";
		}
		return false;
	}
}

bool SceneGraphFacade::addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forcedId) {
	bool operationSucceeded = false;
	bool idIsOk = false;
	Id id;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(parentId);
	Node::NodePtr node = tmpNode.lock();

	Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(node);

	if (forcedId && (parentGroup != 0)) { // NOTE: As we are handling existing IDs we need to be careful with removing them from the pool
		if( (!doesIdExist(assignedId)) && (idGenerator->removeIdFromPool(assignedId)) ) {
			id = assignedId;
			idIsOk = true;
		} else {
			LOG(WARNING) << "Forced ID " << assignedId << " cannot be assigend. Probably another object with that ID exists already!";
			idIsOk = false;
		}
	} else {
		id = idGenerator->getNextValidId();
		idIsOk = true;
	}

	if ((parentGroup != 0) && (idIsOk)) {
		rsg::UncertainTransform::UncertainTransformPtr newTransform(new UncertainTransform());
		newTransform->setId(id);
		newTransform->setAttributes(attributes);
		newTransform->insertTransform(transform, uncertainty, timeStamp);
		parentGroup->addChild(newTransform);
		assignedId = newTransform->getId();
		idLookUpTable.insert(std::make_pair(newTransform->getId(), newTransform));
		operationSucceeded = true;
	}

	/* Call all observers depending on the given policy in case an error occured */
	if (operationSucceeded || callObserversEvenIfErrorsOccurred) {
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			Id assignedIdcopy = assignedId; // prevent that observer might change this....
			(*observerIterator)->addUncertainTransformNode(parentId, assignedIdcopy, attributes, transform, uncertainty, timeStamp);
		}
	}

	if(operationSucceeded) {
		return true;
	} else {
		if (idIsOk) { //If ID was OK but operation did not succeeded, then it is because of the parent...
			LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new uncertain transform as a child of it.";
		}
		return false;
	}
	return false;
}

bool SceneGraphFacade::addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId) {
	bool operationSucceeded = false;
	bool idIsOk = false;
	Id id;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(parentId);
	Node::NodePtr node = tmpNode.lock();
	Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(node);

	if (forcedId && (parentGroup != 0)) { // NOTE: As we are handling existing IDs we need to be careful with removing them from the pool
		if( (!doesIdExist(assignedId)) && (idGenerator->removeIdFromPool(assignedId)) ) {
			id = assignedId;
			idIsOk = true;
		} else {
			LOG(WARNING) << "Forced ID " << assignedId << " cannot be assigend. Probably another object with that ID exists already!";
			idIsOk = false;
		}
	} else {
		id = idGenerator->getNextValidId();
		idIsOk = true;
	}

	if ((parentGroup != 0) && (idIsOk)) {
		GeometricNode::GeometricNodePtr newGeometricNode(new GeometricNode());
		newGeometricNode->setId(id);
		newGeometricNode->setAttributes(attributes);
		newGeometricNode->setShape(shape);
		newGeometricNode->setTimeStamp(timeStamp);
		parentGroup->addChild(newGeometricNode);
		assignedId = newGeometricNode->getId();
		idLookUpTable.insert(std::make_pair(newGeometricNode->getId(), newGeometricNode));
		operationSucceeded = true;
	}

	/* Call all observers depending on the given policy in case an error occured */
	if (operationSucceeded || callObserversEvenIfErrorsOccurred) {
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			Id assignedIdcopy = assignedId; // prevent that observer might change this....
			(*observerIterator)->addGeometricNode(parentId, assignedIdcopy, attributes, shape, timeStamp);
		}
	}

	if(operationSucceeded) {
		return true;
	} else {
		if (idIsOk) { //If ID was OK but operation did not succeeded, then it is because of the parent...
			LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new geometric node as a child of it.";
		}
		return false;
	}
}


bool SceneGraphFacade::setNodeAttributes(Id id, vector<Attribute> newAttributes) {
	bool operationSucceeded = false;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	if (node != 0) {
		node->setAttributes(newAttributes);
		operationSucceeded = true;
	}

	/* Call all observers depending on the given policy in case an error occured */
	if (operationSucceeded || callObserversEvenIfErrorsOccurred) {
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			Id assignedIdcopy = id; // prevent that observer might change this....
			(*observerIterator)->setNodeAttributes(assignedIdcopy, newAttributes);
		}
	}

	if (operationSucceeded) {
		return true;
	} else {
		return false;
	}
}

bool SceneGraphFacade::setTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
	bool operationSucceeded = false;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	rsg::Transform::TransformPtr transformNode = boost::dynamic_pointer_cast<rsg::Transform>(node);
	if (transformNode != 0) {
		operationSucceeded = transformNode->insertTransform(transform, timeStamp);
	}

	/* Call all observers depending on the given policy in case an error occured */
	if (operationSucceeded || callObserversEvenIfErrorsOccurred) {
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			(*observerIterator)->setTransform(id, transform, timeStamp);
		}
	}

	if (operationSucceeded) {
		return true;
	} else {
		LOG(ERROR) << "Node with ID " << id << " is not a transform. Cannot set new transform data.";
		return false;
	}
}

bool SceneGraphFacade::setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp) {
	bool operationSucceeded = false;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	rsg::UncertainTransform::UncertainTransformPtr transformNode = boost::dynamic_pointer_cast<rsg::UncertainTransform>(node);
	if (transformNode != 0) {
		operationSucceeded = transformNode->insertTransform(transform, uncertainty, timeStamp);
	}

	/* Call all observers depending on the given policy in case an error occured */
	if (operationSucceeded || callObserversEvenIfErrorsOccurred) {
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			(*observerIterator)->setUncertainTransform(id, transform, uncertainty, timeStamp);
		}
	}

	if (operationSucceeded) {
		return true;
	} else {
		LOG(ERROR) << "Node with ID " << id << " is not an uncertain transform. Cannot set new transform and uncertainy data.";
		return false;
	}
	return false;
}

bool SceneGraphFacade::deleteNode(Id id) {
	bool operationSucceeded = false;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	if (node != 0) {
		if (node->getNumberOfParents() == 0) { // oops we are trying to delete the root node, but this on has no parents...
			assert (id == getRootId()); // just to be sure something really strange did not happend...
			LOG(WARNING) << "The root node with ID " << id << " cannot be deleted.";
			operationSucceeded = false;

		} else {

			/*
			 * so we found the handle to the current node; now we will invoke
			 * the according delete function for every parent
			 */
			while (node->getNumberOfParents() > 0) { //NOTE: node->getNumberOfParents() will decrease within every iteration...
				unsigned int i = 0;
				rsg::Node* parentNode;
				parentNode = node->getParent(i);
				Group* parentGroup =  dynamic_cast<Group*>(parentNode);
				if (parentGroup != 0 ) {
					parentGroup->removeChild(node);
				} else {
					assert(false); // actually parents need to be groups otherwise sth. really went wrong
				}
			}
			idLookUpTable.erase(id); //erase by ID (if not done here there would be orphaned IDs)
			// TODO: do we have to delete children?
			operationSucceeded = true;
		}
	}

	/* Call all observers depending on the given policy in case an error occured */
	if (operationSucceeded || callObserversEvenIfErrorsOccurred) {
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			(*observerIterator)->deleteNode(id);
		}
	}

	if (operationSucceeded) {
		return true;
	} else {
		return false;
	}
}

bool SceneGraphFacade::addParent(Id id, Id parentId) {
	bool operationSucceeded = false;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	Node::NodeWeakPtr tmpParentNode = findNodeRecerence(parentId);
	Node::NodePtr parentNode = tmpParentNode.lock();
	Group::GroupPtr parentGroup = boost::dynamic_pointer_cast<Group>(parentNode);

	if (parentGroup != 0 && node != 0 && (id != parentId)) {
		parentGroup->addChild(node);
		operationSucceeded = true;
	}

	/* Call all observers depending on the given policy in case an error occured */
	if (operationSucceeded || callObserversEvenIfErrorsOccurred) {
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			(*observerIterator)->addParent(id, parentId);
		}
	}

	if (operationSucceeded) {
		return true;
	} else {
		if (id == parentId) {
			LOG(ERROR) << "Parent with ID " << parentId << " and child ID " << id << " are identical. Cannot add a new parent-child relation.";
		} else {
			LOG(ERROR) << "Parent with ID " << parentId << " is not a group. Cannot add a new parent-child relation.";
		}

		return false;
	}
}

bool SceneGraphFacade::removeParent(Id id, Id parentId) {
	bool operationSucceeded = false;
	Node::NodeWeakPtr tmpNode = findNodeRecerence(id);
	Node::NodePtr node = tmpNode.lock();
	if (node != 0) {
		if (node->getNumberOfParents() == 0) { // oops we are trying to delete the root node, but this on has no parents...
			assert (id == getRootId()); // just to be sure something really strange did not happend...
			LOG(WARNING) << "The root node with ID " << id << " has no parents that could be removed.";
			operationSucceeded = false;

		} else {

			/*
			 * so we found the handle to the current node; now we will look
			 * for respective parent and delete that
			 */
			for (unsigned int currentParentID = 0; currentParentID < node->getNumberOfParents(); ++currentParentID) {//NOTE: node->getNumberOfParents() will decrease within every iteration...
				//unsigned int i = 0;
				rsg::Node* parentNode;
				parentNode = node->getParent(currentParentID);
				if (parentNode->getId() == parentId) { // (Finally) we found the parent of interest.
					Group* parentGroup =  dynamic_cast<Group*>(parentNode);
					if (parentGroup != 0 ) {
						parentGroup->removeChild(node);
						operationSucceeded = true;
					} else {
						assert(false); // actually parents need to be groups otherwise sth. really went wrong
					}
				}
			}
			// TODO: what abput orphaned IDs?

		}
	}

	/* Call all observers depending on the given policy in case an error occured */
	if (operationSucceeded || callObserversEvenIfErrorsOccurred) {
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			(*observerIterator)->removeParent(id, parentId);
		}
	}

	if (operationSucceeded) {
		return true;
	} else {
		return false;
	};
}

bool SceneGraphFacade::attachUpdateObserver(ISceneGraphUpdateObserver* observer) {
	assert(observer != 0);
	updateObservers.push_back(observer);
	return true;
}

bool SceneGraphFacade::detachUpdateObserver(ISceneGraphUpdateObserver* observer) {
	assert(observer != 0);
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator = std::find(updateObservers.begin(), updateObservers.end(), observer);
    if (observerIterator!=updateObservers.end()) {
    	updateObservers.erase(observerIterator);
    	return true;
    }
    LOG(ERROR) << "Cannot detach update observer. Provided reference does not match with any in the observers list.";
	return false;
}

bool SceneGraphFacade::isCallObserversEvenIfErrorsOccurred() const {
	return callObserversEvenIfErrorsOccurred;
}

void SceneGraphFacade::setCallObserversEvenIfErrorsOccurred(
		bool callObserversEvenIfErrorsOccurred) {
	this->callObserversEvenIfErrorsOccurred =
			callObserversEvenIfErrorsOccurred;
}

bool SceneGraphFacade::executeGraphTraverser(INodeVisitor* visitor, Id subgraphId) {
	Node::NodeWeakPtr tmpNode = findNodeRecerence(subgraphId);
	Node::NodePtr node = tmpNode.lock();
	if (node != 0) {
		node->accept(visitor);
		return true;
	}
	return false;
}

Node::NodeWeakPtr SceneGraphFacade::findNodeRecerence(Id id) {
	nodeIterator = idLookUpTable.find(id);
	if (nodeIterator != idLookUpTable.end()) { //TODO multiple IDs?
		Node::NodePtr tmpNodeHandle = nodeIterator->second.lock();
		if(tmpNodeHandle !=0) {
//			assert (id == tmpNodeHandle->getId()); // Otherwise something really went wrong while maintaining IDs...
			if(id != tmpNodeHandle->getId()) {
				LOG(ERROR) << "id != tmpNodeHandle->getId() " << id << "; " << tmpNodeHandle->getId();
				return Node::NodeWeakPtr();
			}
			return nodeIterator->second;
		}
		LOG(WARNING) << "ID " << id << " seems to be orphaned. Possibly its node has been deleted earlier.";
	}
	LOG(WARNING) << "Scene graph does not contain a node with ID " << id;
	return Node::NodeWeakPtr(); // should be kind of null...
}

bool SceneGraphFacade::doesIdExist(Id id) {
	nodeIterator = idLookUpTable.find(id);
	if (nodeIterator != idLookUpTable.end()) {
		return true;
	}
	return false;
}

} // namespace brics_3d::RSG

} // namespace brics_3d

/* EOF */

