/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2016, KU Leuven
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

#include "SemanticContextUpdateFilter.h"
#include "brics_3d/core/Logger.h"

namespace brics_3d {
namespace rsg {

SemanticContextUpdateFilter::SemanticContextUpdateFilter(SceneGraphFacade* scene) : scene(scene) {
	this->nameSpaceIdentifier = "unknown_namespace";
	assert(this->scene != 0);
}

SemanticContextUpdateFilter::~SemanticContextUpdateFilter() {

}

bool SemanticContextUpdateFilter::addNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:addNode creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addNode(parentId, assignedId, attributes, forcedId);
	}
	return true;
}

bool SemanticContextUpdateFilter::addGroup(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:addGroup creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addGroup(parentId, assignedId, attributes, forcedId);
	}
	return true;
}

bool SemanticContextUpdateFilter::addTransformNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:addTransformNode creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addTransformNode(parentId, assignedId, attributes, transform, timeStamp, forcedId);
	}

	return true;

}

bool SemanticContextUpdateFilter::addUncertainTransformNode(Id parentId,
		Id& assignedId, vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:addUncertainTransformNode creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addUncertainTransformNode(parentId, assignedId, attributes, transform, uncertainty, timeStamp, forcedId);
	}

	return true;
}


bool SemanticContextUpdateFilter::addGeometricNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, Shape::ShapePtr shape,
		TimeStamp timeStamp, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:addGeometricNode creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}


	/* Inform related observer(s) */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addGeometricNode(parentId, assignedId, attributes, shape, timeStamp, forcedId);
	}

	return true;
}

bool SemanticContextUpdateFilter::addRemoteRootNode(Id rootId, vector<Attribute> attributes) {

	/* THIS MUST NOT BE FILTERED */

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addRemoteRootNode(rootId, attributes);
	}

	return true;
}

bool SemanticContextUpdateFilter::addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:addConnection creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addConnection(parentId, assignedId, attributes, sourceIds, targetIds, start, end, forcedId);
	}
	return true;
}

bool SemanticContextUpdateFilter::setNodeAttributes(Id id,
		vector<Attribute> newAttributes, TimeStamp timeStamp) {

	/* NOTE: here we check the _existing_ attributes to be consistent with other update functions. Not the new ones. */
	vector<Attribute> attributes;
	if(!scene->getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:setNodeAttributes cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:setNodeAttributes update is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->setNodeAttributes(id, newAttributes, timeStamp);
	}
	return true;
}

bool SemanticContextUpdateFilter::setTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp) {

	vector<Attribute> attributes;
	if(!scene->getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:setTransform cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:setTransform update is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Inform related observer(s) */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->setTransform(id, transform, timeStamp);
	}

	return true;
}

bool SemanticContextUpdateFilter::setUncertainTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp) {

	vector<Attribute> attributes;
	if(!scene->getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:setUncertainTransform cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:setUncertainTransform update is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Inform related observer(s) */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->setUncertainTransform(id, transform, uncertainty, timeStamp);
	}

	return true;
}

bool SemanticContextUpdateFilter::deleteNode(Id id) {
	/* Call _all_ observers  */

	vector<Attribute> attributes;
	if(!scene->getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:deleteNode cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:deleteNode deletion is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->deleteNode(id);
	}
	return true;
}

bool SemanticContextUpdateFilter::addParent(Id id, Id parentId) {

	vector<Attribute> attributes;
	if(!scene->getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:addParent cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:addParent creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addParent(id, parentId);
	}
	return true;
}

bool SemanticContextUpdateFilter::removeParent(Id id, Id parentId) {

	vector<Attribute> attributes;
	if(!scene->getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:removeParent cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "SemanticContextUpdateFilter:removeParent deletion is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->removeParent(id, parentId);
	}
	return true;
}

bool SemanticContextUpdateFilter::attachUpdateObserver(
		ISceneGraphUpdateObserver* observer) {
	assert(observer != 0);
	updateObservers.push_back(observer);
	return true;
}

bool SemanticContextUpdateFilter::detachUpdateObserver(
		ISceneGraphUpdateObserver* observer) {

	assert(observer != 0);
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator = std::find(updateObservers.begin(), updateObservers.end(), observer);
    if (observerIterator!=updateObservers.end()) {
    	updateObservers.erase(observerIterator);
    	return true;
    }
    LOG(ERROR) << "Cannot detach update observer. Provided reference does not match with any in the observers list.";
	return false;
}

const string& SemanticContextUpdateFilter::getNameSpaceIdentifier() const {
	return nameSpaceIdentifier;
}

void SemanticContextUpdateFilter::setNameSpaceIdentifier(
		const string& nameSpaceIdentifier) {
	this->nameSpaceIdentifier = nameSpaceIdentifier;
	this->query =  "^"  + nameSpaceIdentifier + ":.*"; // wildcard = ^.*
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
