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

#include "GraphConstraintUpdateFilter.h"
#include "brics_3d/core/Logger.h"
#include "brics_3d/util/Timer.h"

namespace brics_3d {
namespace rsg {

GraphConstraintUpdateFilter::GraphConstraintUpdateFilter(WorldModel* wm, UpdateMode mode) : mode(mode), wm(wm) {
	this->nameSpaceIdentifier = "unknown_namespace";
	assert(this->wm != 0);
}

GraphConstraintUpdateFilter::~GraphConstraintUpdateFilter() {

}

bool GraphConstraintUpdateFilter::addNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {

	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {

		Duration duration;
		Duration allowedDuration;
		double distanceInMeters = 0.0;
		GraphConstraint::Type type = GraphConstraint::Node;

		switch (mode) {
			case SENDER:

				/* check if it applies to sender */
				if(it->action != GraphConstraint::SEND) {
					break; // this constraint does not apply at all
				}


				switch (it->nodeConstraint) {

					/* simple constraints first */
					case GraphConstraint::UNDEFINED_NODE_CONSTRAINT:

						if((it->qualifier == GraphConstraint::NO) && (it->type == type)) {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:addNode creation is skipped because no types " << type << " are allowed";
							return false;
						} else if((it->qualifier == GraphConstraint::NO) && (it->type == GraphConstraint::Atom)) {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:addNode creation is skipped because no types " << GraphConstraint::Atom << " are allowed";
							return false;
						} else if((it->qualifier == GraphConstraint::ONLY) && (it->type != GraphConstraint::Node)) {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:addNode creation is skipped because only types " << type << " are allowed";
							return false;
						}

						break;



					/* complex constraints */
					/*
					 * (( with freq (<|=|>) [0-9]+ (Hz|kHz))|\
					 *  ( with lod (<|=|>) [0-9]+)|\
					 *  ( with dist (<|=|>) [0-9]+ (mm|cm|m|km) from (me|([a-fA-F0-9]{8}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{12}))|\
					 *  ( from context [a-zA-Z0-9]+)|\
					 *  ( contained in ([a-fA-F0-9]{8}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{12}))
					 */
					case GraphConstraint::FREQUENCY:

						duration = wm->now() - lastSendType[it->type];
						allowedDuration = 1.0 / Units::frequencyToHertz(it->value, it->freqUnit); // convert Hz to seconds.

						if( 	((it->qualifier == GraphConstraint::NO) && (it->type == type)) ||
								((it->qualifier == GraphConstraint::NO) && (it->type == GraphConstraint::Atom)) ) {

							switch (it->comparision) {
								case GraphConstraint::EQ:
									if(duration == allowedDuration) {
										LOG(DEBUG) << "GraphConstraintUpdateFilter:addNode creation is skipped because no types "
												<< it->type << " with duration = "<< duration.getSeconds() <<"[s] are allowed";
										return false;
									}
									break;
								case GraphConstraint::LT:
									if(duration > allowedDuration) {
										LOG(DEBUG) << "GraphConstraintUpdateFilter:addNode creation is skipped because no types "
												<< it->type << " with duration > "<< duration.getSeconds() <<"[s] are allowed";
										return false;
									}

									break;
								case GraphConstraint::GT:
									if(duration < allowedDuration) {
										LOG(DEBUG) << "GraphConstraintUpdateFilter:addNode creation is skipped because no types "
												<< it->type << " with duration < "<< duration.getSeconds() <<"[s] are allowed";
										return false;
									}
									break;

								case GraphConstraint::UNDEFINED_OPERATOR:
									LOG(ERROR) << "GraphConstraintUpdateFilter:addNode creation due to an undefined operator in a freqeuncy constraint.";
									return false;
							}


						} else if((it->qualifier == GraphConstraint::ONLY) && (it->type != GraphConstraint::Node)) {

							switch (it->comparision) {
								case GraphConstraint::EQ:
									if(duration == allowedDuration) {
										continue;
									}
									break;
								case GraphConstraint::LT:
									if(duration < allowedDuration) {
										LOG(DEBUG) << "GraphConstraintUpdateFilter:addNode creation is skipped because only types "
												<< it->type << " with duration > "<< duration.getSeconds() <<"[s] are allowed";
										return false;
									}

									break;
								case GraphConstraint::GT:
									if(duration > allowedDuration) {
										LOG(DEBUG) << "GraphConstraintUpdateFilter:addNode creation is skipped because only types "
												<< it->type << " with duration < "<< duration.getSeconds() <<"[s] are allowed";
										return false;
									}
									break;

								case GraphConstraint::UNDEFINED_OPERATOR:
									LOG(ERROR) << "GraphConstraintUpdateFilter:addNode creation due to an undefined operator in a freqeuncy constraint.";
									return false;

							}
						}

						break;

					case GraphConstraint::LOD:
						// only applies to geometric nodes
						break;

					case GraphConstraint::DISTANCE:



						// only applies to geometric nodes
						break;

					default:

						break;

				}


				break;

			case RECEIVER:

				break;

			default:
				LOG(ERROR) << "GraphConstraintUpdateFilter:addNode creation is skipped because the mode is undefined";
				return false;
				break;
		}
	}

	/* Memorize last invocation */
	lastSendType[GraphConstraint::Node] = wm->now();
	lastSendType[GraphConstraint::Atom] = wm->now();

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addNode(parentId, assignedId, attributes, forcedId);
	}
	return true;
}

bool GraphConstraintUpdateFilter::addGroup(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:addGroup creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Memorize last invocation */
	lastSendType[GraphConstraint::Group] = wm->now();
	lastSendType[GraphConstraint::Atom] = wm->now();


	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addGroup(parentId, assignedId, attributes, forcedId);
	}
	return true;
}

bool GraphConstraintUpdateFilter::addTransformNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:addTransformNode creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Memorize last invocation */
	lastSendType[GraphConstraint::Transform] = wm->now();
	lastSendType[GraphConstraint::Atom] = wm->now();

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addTransformNode(parentId, assignedId, attributes, transform, timeStamp, forcedId);
	}

	return true;

}

bool GraphConstraintUpdateFilter::addUncertainTransformNode(Id parentId,
		Id& assignedId, vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:addUncertainTransformNode creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Memorize last invocation */
	lastSendType[GraphConstraint::Transform] = wm->now();
	lastSendType[GraphConstraint::Atom] = wm->now();

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addUncertainTransformNode(parentId, assignedId, attributes, transform, uncertainty, timeStamp, forcedId);
	}

	return true;
}


bool GraphConstraintUpdateFilter::addGeometricNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, Shape::ShapePtr shape,
		TimeStamp timeStamp, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:addGeometricNode creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Memorize last invocation */
	lastSendType[GraphConstraint::GeometricNode] = wm->now();
	lastSendType[GraphConstraint::Atom] = wm->now();

	switch (shape->getShapeType()) {
		case Shape::Sphere:
			lastSendType[GraphConstraint::Sphere] = wm->now();
			break;
		case Shape::Cylinder:
			lastSendType[GraphConstraint::Cylinder] = wm->now();
			break;
		case Shape::Box:
			lastSendType[GraphConstraint::Box] = wm->now();
			break;
		case Shape::Mesh:
			lastSendType[GraphConstraint::Mesh] = wm->now();
			break;
		case Shape::PointCloud:
			lastSendType[GraphConstraint::PointCloud] = wm->now();
			break;

		default:
			break;
	}

	/* Inform related observer(s) */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addGeometricNode(parentId, assignedId, attributes, shape, timeStamp, forcedId);
	}

	return true;
}

bool GraphConstraintUpdateFilter::addRemoteRootNode(Id rootId, vector<Attribute> attributes) {

	/* THIS MUST NOT BE FILTERED */

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addRemoteRootNode(rootId, attributes);
	}

	return true;
}

bool GraphConstraintUpdateFilter::addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId) {

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:addConnection creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Memorize last invocation */
	lastSendType[GraphConstraint::Connection] = wm->now();
	lastSendType[GraphConstraint::Atom] = wm->now();

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addConnection(parentId, assignedId, attributes, sourceIds, targetIds, start, end, forcedId);
	}
	return true;
}

bool GraphConstraintUpdateFilter::setNodeAttributes(Id id,
		vector<Attribute> newAttributes, TimeStamp timeStamp) {

	/* NOTE: here we check the _existing_ attributes to be consistent with other update functions. Not the new ones. */
	vector<Attribute> attributes;
	if(!wm->scene.getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:setNodeAttributes cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:setNodeAttributes update is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->setNodeAttributes(id, newAttributes, timeStamp);
	}
	return true;
}

bool GraphConstraintUpdateFilter::setTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp) {

	vector<Attribute> attributes;
	if(!wm->scene.getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:setTransform cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:setTransform update is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Memorize last invocation */
	lastSendType[GraphConstraint::Transform] = wm->now();

	/* Inform related observer(s) */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->setTransform(id, transform, timeStamp);
	}

	return true;
}

bool GraphConstraintUpdateFilter::setUncertainTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp) {

	vector<Attribute> attributes;
	if(!wm->scene.getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:setUncertainTransform cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:setUncertainTransform update is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Memorize last invocation */
	lastSendType[GraphConstraint::Transform] = wm->now();

	/* Inform related observer(s) */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->setUncertainTransform(id, transform, uncertainty, timeStamp);
	}

	return true;
}

bool GraphConstraintUpdateFilter::deleteNode(Id id) {
	/* Call _all_ observers  */

	vector<Attribute> attributes;
	if(!wm->scene.getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:deleteNode cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:deleteNode deletion is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->deleteNode(id);
	}
	return true;
}

bool GraphConstraintUpdateFilter::addParent(Id id, Id parentId) {

	vector<Attribute> attributes;
	if(!wm->scene.getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:addParent cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:addParent creation is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addParent(id, parentId);
	}
	return true;
}

bool GraphConstraintUpdateFilter::removeParent(Id id, Id parentId) {

	vector<Attribute> attributes;
	if(!wm->scene.getNodeAttributes(id, attributes)) {
		LOG(ERROR) << "semanticContextUpdateFilter:removeParent cannot query existing attributes for id " << id << " Skipping update.";
		return false;
	}

	if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
		LOG(DEBUG) << "GraphConstraintUpdateFilter:removeParent deletion is skipped because attributes contain (" << query << ", *)";
		return false;
	}

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->removeParent(id, parentId);
	}
	return true;
}

bool GraphConstraintUpdateFilter::attachUpdateObserver(
		ISceneGraphUpdateObserver* observer) {
	assert(observer != 0);
	updateObservers.push_back(observer);
	return true;
}

bool GraphConstraintUpdateFilter::detachUpdateObserver(
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

const string& GraphConstraintUpdateFilter::getNameSpaceIdentifier() const {
	return nameSpaceIdentifier;
}

void GraphConstraintUpdateFilter::setNameSpaceIdentifier(
		const string& nameSpaceIdentifier) {
	this->nameSpaceIdentifier = nameSpaceIdentifier;
	this->query =  "^"  + nameSpaceIdentifier + ":.*"; // wildcard = ^.*
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
