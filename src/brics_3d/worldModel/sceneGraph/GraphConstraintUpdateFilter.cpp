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
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/util/Timer.h"
#include "SubGraphChecker.h"

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
		if (!checkConstraint(*it, GraphConstraint::Node, 0, 0, 0, assignedId, attributes)) {
			LOG(DEBUG) << "GraphConstraintUpdateFilter::addNode is skipped because a constraint does not hold.";
			return false;
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

	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {
		if (!checkConstraint(*it, GraphConstraint::Group, 0, 0, 0, assignedId, attributes)) {
			LOG(DEBUG) << "GraphConstraintUpdateFilter::addGroup is skipped because a constraint does not hold.";
			return false;
		}
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

	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {
		if (!checkConstraint(*it, GraphConstraint::Transform, 0, 0, 0, assignedId, attributes)) {
			LOG(DEBUG) << "GraphConstraintUpdateFilter::addTransformNode is skipped because a constraint does not hold.";
			return false;
		}
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

	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {
		if (!checkConstraint(*it, GraphConstraint::Transform, 0, 0, 0, assignedId, attributes)) {
			LOG(DEBUG) << "GraphConstraintUpdateFilter::addUncertainTransformNode is skipped because a constraint does not hold.";
			return false;
		}
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

	GraphConstraint::Type shapeType;
	switch (shape->getShapeType()) {
		case Shape::Sphere:
			shapeType = GraphConstraint::Sphere;
			break;
		case Shape::Cylinder:
			shapeType = GraphConstraint::Cylinder;
			break;
		case Shape::Box:
			shapeType = GraphConstraint::Box;
			break;
		case Shape::Mesh:
			shapeType = GraphConstraint::Mesh;
			break;
		case Shape::PointCloud:
			shapeType = GraphConstraint::PointCloud;
			break;
		default:
			break;
	}

	/* TODO calculate LOD */

	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {
		if (!checkConstraint(*it, GraphConstraint::GeometricNode, 0, 0, 0, assignedId, attributes)) {
			LOG(DEBUG) << "GraphConstraintUpdateFilter::addNode is skipped because a constraint does not hold for GeometricNode.";
			return false;
		}
		if (!checkConstraint(*it, shapeType, 0, 0, 0, assignedId, attributes)) {
			LOG(DEBUG) << "GraphConstraintUpdateFilter::addNode is skipped because a constraint does not hold for Shape type " << shapeType << ".";
			return false;
		}
	}

	/* Memorize last invocation */
	lastSendType[shapeType] = wm->now();
	lastSendType[GraphConstraint::GeometricNode] = wm->now();
	lastSendType[GraphConstraint::Atom] = wm->now();



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

	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {
		if (!checkConstraint(*it, GraphConstraint::Connection, 0, 0, 0, assignedId, attributes)) {
			LOG(DEBUG) << "GraphConstraintUpdateFilter::addConnection is skipped because a constraint does not hold.";
			return false;
		}
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


	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {
		if (!checkConstraint(*it, GraphConstraint::Atom, 0, 0, 0, id, attributes)) { // assignedID?
			LOG(DEBUG) << "GraphConstraintUpdateFilter::addNode is skipped because a constraint does not hold.";
			return false;
		}
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

	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {
		if (!checkConstraint(*it, GraphConstraint::Transform, 0, 0, 0, id, attributes)) { // assignedID?
			LOG(DEBUG) << "GraphConstraintUpdateFilter::setTransform is skipped because a constraint does not hold.";
			return false;
		}
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

	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {
		if (!checkConstraint(*it, GraphConstraint::Transform, 0, 0, 0, id, attributes)) { // assignedID?
			LOG(DEBUG) << "GraphConstraintUpdateFilter::setUncertainTransform is skipped because a constraint does not hold.";
			return false;
		}
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

	/*
	 *  This cannot be constrained, because we cannot make queries on it: it does not exist any more...
	 */

	/* Call _all_ observers  */
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

	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {
		if (!checkConstraint(*it, GraphConstraint::Atom, 0, 0, 0, id, attributes)) { // assignedID?
			LOG(DEBUG) << "GraphConstraintUpdateFilter::addParent is skipped because a constraint does not hold.";
			return false;
		}
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

	std::vector<GraphConstraint>::iterator it;
	for (it = constraints.begin(); it != constraints.end(); ++it) {
		if (!checkConstraint(*it, GraphConstraint::Atom, 0, 0, 0, id, attributes)) { // assignedID?
			LOG(DEBUG) << "GraphConstraintUpdateFilter::removeParent is skipped because a constraint does not hold.";
			return false;
		}
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

bool GraphConstraintUpdateFilter::checkConstraint(GraphConstraint constraint,
		GraphConstraint::Type type, double frequencyInHz, double distanceInMeters,
		double lod, Id assignedId, vector<Attribute> attributes) {

	//GraphConstraint::Type type = GraphConstraint::Node;
	double allowedFrequencyInHz = 0.0;
	double allowedDistanceInMeters = 0.0;
	double allowedLod = 0.0;
	Id referenceNode;
	HomogeneousMatrix44::IHomogeneousMatrix44Ptr tf;
	SubGraphChecker subGraph(assignedId);
	bool isContainedIn = false;
	string query = "*";

	switch (mode) {
		case SENDER:

			/* check if it applies to sender */
			if(constraint.action != GraphConstraint::SEND) {
				break; // this constraint does not apply at all
			}


			switch (constraint.nodeConstraint) {

				/* simple constraints first */
				case GraphConstraint::NONE:

					if( (constraint.qualifier == GraphConstraint::NO) && (constraint.type == type) ) {
						LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint creation is false because no types " << type << " are allowed";
						return false;
					} else if( (constraint.qualifier == GraphConstraint::NO) && (constraint.type == GraphConstraint::Atom) ) {
						LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint creation is false because no types " << GraphConstraint::Atom << " are allowed";
						return false;
					} else if( (constraint.qualifier == GraphConstraint::ONLY) && (constraint.type == GraphConstraint::Atom) ) {
						LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint creation is true because all types, even " << type << " are allowed";
						return true;
					} else if( (constraint.qualifier == GraphConstraint::ONLY) && (constraint.type != type) ) {
						LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint creation is false because only types " << type << " are allowed";
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


					frequencyInHz = 1.0/(wm->now() - lastSendType[constraint.type]).getSeconds();
					allowedFrequencyInHz = Units::frequencyToHertz(constraint.value, constraint.freqUnit); // convert Hz to seconds.

					return checkComparision(constraint, type, frequencyInHz, allowedFrequencyInHz, "frequency");

					break;

				case GraphConstraint::LOD:

					// only applies to geometric nodes
					allowedLod = constraint.value;
					return checkComparision(constraint, type, lod, allowedLod, "lod");

					break;

				case GraphConstraint::DISTANCE:

					/* get reference node */
					if(constraint.isMe) {
						referenceNode = wm->getRootNodeId();
					} else {
						referenceNode = constraint.node;
					}

					/* compute distance */
					if(!wm->scene.getTransformForNode(assignedId, referenceNode, wm->now(), tf)) { // TODO find a way for to get the distance for non existing nodes (e.g. parent)
						LOG(WARNING) << "GraphConstraintUpdateFilter:checkConstraint is false because the distance between two nodes could not been derived.";
								return false;
					}
					HomogeneousMatrix44::matrixToDistance(tf, distanceInMeters); // in meters ?!?

					/* check it */
					allowedDistanceInMeters = Units::distanceToMeters(constraint.value, constraint.distUnit);
					return checkComparision(constraint, type, distanceInMeters, allowedDistanceInMeters, "distance");

					break;

				case GraphConstraint::CONTEXT:

					query =  "^"  + constraint.context + ":.*"; // wildcard = ^.*
					if(constraint.qualifier == GraphConstraint::ONLY) {

						if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint is true because attributes contain (only) (" << constraint.context << ", *)";
							return true;
						} else {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint is false because attributes do not contain (only) (" << constraint.context << ", *)";
							return false;
						}

					} else if (constraint.qualifier == GraphConstraint::NO) {

						if (attributeListContainsAttribute(attributes, Attribute(query, "*"))) {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint is false because attributes contain (no) (" << constraint.context << ", *)";
							return false;
						} else {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint is true because attributes do not contain (no) (" << constraint.context << ", *)";
							return true;
						}

					}

					break;

				case GraphConstraint::CONTAINMENT:

					/* get reference node */
					if(constraint.isMe) {
						referenceNode = wm->getRootNodeId();
					} else {
						referenceNode = constraint.node;
					}

					subGraph.reset(referenceNode); // NOTE: upwards traversal
					wm->scene.executeGraphTraverser(&subGraph, assignedId);
					isContainedIn = subGraph.nodeIsInSubGraph();

					if(constraint.qualifier == GraphConstraint::ONLY) {

						if (isContainedIn) {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint is true because node is contained in " << referenceNode;
							return true;
						} else {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint is false because node is not contained in " << referenceNode;
							return false;
						}

					} else if (constraint.qualifier == GraphConstraint::NO) {

						if (isContainedIn) {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint is false because node is contained in " << referenceNode << " although it should not.";
							return false;
						} else {
							LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint is true because node is not contained in " << referenceNode << " as it should.";
							return true;
						}

					}


					break;

				case GraphConstraint::UNDEFINED_NODE_CONSTRAINT:

					LOG(DEBUG) << "GraphConstraintUpdateFilter:checkConstraint is false because the nodeConstraint is undefined. Do you actually mean NONE instead?";
					break;

				default:

					break;

			}


			break;

		case RECEIVER:

			break;

		default:
			LOG(ERROR) << "GraphConstraintUpdateFilter:checkConstraint is false because the I/O mode is undefined.";
			return false;
			break;
	}

	return true;
}

bool GraphConstraintUpdateFilter::checkComparision(GraphConstraint constraint, GraphConstraint::Type type, double value, double allowedValue, string tag) {

	// NO
	if( 	((constraint.qualifier == GraphConstraint::NO) && (constraint.type == type)) ||
			((constraint.qualifier == GraphConstraint::NO) && (constraint.type == GraphConstraint::Atom)) ) {

		switch (constraint.comparision) {
		case GraphConstraint::EQ:
			if(value == allowedValue) {
				LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is false because no types "
						<< constraint.type << " with " << tag <<" " << value << " == " << allowedValue <<" are allowed";
				return false;
			} else {
				LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is true because types "
						<< constraint.type << " with " << tag <<" " << value << " == " << allowedValue <<" are allowed";
				return true;
			}

			break;

		case GraphConstraint::LT:
			if(value < allowedValue) {
				LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is false because no types "
						<< constraint.type << " with " << tag <<" " << value << " < " << allowedValue <<" are allowed";
				return false;
			} else {
				LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is true because types "
						<< constraint.type << " with " << tag <<" " << value << " > " << allowedValue <<" are allowed";
				return true;
			}

			break;

		case GraphConstraint::GT:
			if(value > allowedValue) {
				LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is false because no types "
						<< constraint.type << " with " << tag <<" " << value << " > " << allowedValue <<" are allowed";
				return false;
			} else {
				LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is true because types "
						<< constraint.type << " with " << tag <<" " << value << " < " << allowedValue <<" are allowed";
				return true;
			}

			break;

		case GraphConstraint::UNDEFINED_OPERATOR:
			LOG(ERROR) << "GraphConstraintUpdateFilter:checkComparision is false due to an undefined operator in a " << tag << " constraint.";
			return false;

		default:

			LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is true because types "
			<< constraint.type << " with " << tag <<" " << value << " of " << allowedValue <<" are allowed";
			return true;
		}

	// ONLY
	} else if( ((constraint.qualifier == GraphConstraint::ONLY) && (constraint.type == type)) ||
			    ((constraint.qualifier == GraphConstraint::ONLY) && (constraint.type == GraphConstraint::Atom)) ) {

		switch (constraint.comparision) {
		case GraphConstraint::EQ:
			if(value == allowedValue) {
				LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is true because only types "
						<< constraint.type << " with " << tag <<" " << value << " == " << allowedValue <<" are allowed.";
				return true;
			}
			break;
		case GraphConstraint::LT:
			if(value < allowedValue) {
				LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is true because only types "
						<< constraint.type << " with " << tag <<" " << value << " < " << allowedValue <<" are allowed.";
				return true;
			}

			break;
		case GraphConstraint::GT:
			if(value > allowedValue) {
				LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is true because only types "
						<< constraint.type << " with " << tag <<" " << value << " > " << allowedValue <<" are allowed.";
				return true;
			}
			break;

		case GraphConstraint::UNDEFINED_OPERATOR:
			LOG(ERROR) << "GraphConstraintUpdateFilter:checkComparision is false due to an undefined operator in a " << tag << " constraint.";
			return false;

		default:

			LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is false because types "
			<< constraint.type << " with " << tag <<" " << value << " of " << allowedValue <<" not allowed";
			return false;

		}



	} else if( constraint.qualifier == GraphConstraint::NO) { // NO and not the above cased
		LOG(DEBUG) << "GraphConstraintUpdateFilter:checkComparision is true because this type "
				<< type   << " is not considered in a this (NO) constraint "
				<< constraint.type << " with " << tag <<" " << value << " <|=|> " << allowedValue;
		return true;
	}

	LOG(ERROR) << "GraphConstraintUpdateFilter:checkComparision is false because no other rule applied to type "
			<< constraint.type << " with " << tag <<" " << value << " <|=|> " << allowedValue;
	return false;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
