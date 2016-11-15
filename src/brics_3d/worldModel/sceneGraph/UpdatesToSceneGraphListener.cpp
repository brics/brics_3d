/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2014, KU Leuven
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

#include "UpdatesToSceneGraphListener.h"
#include <brics_3d/core/Logger.h>


using brics_3d::Logger;

namespace brics_3d {
namespace rsg {

UpdatesToSceneGraphListener::UpdatesToSceneGraphListener() {
	setForcedIdPolicy(ENFORCE_TO_TAKE_OVER_EXISTING_IDS);
}

UpdatesToSceneGraphListener::~UpdatesToSceneGraphListener() {

}

bool UpdatesToSceneGraphListener::addNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {
	bool success = true;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->addNode(parentId, assignedId, attributes, forcedId||forcedIdPolicy);
	}
	return success;
}

bool UpdatesToSceneGraphListener::addGroup(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {
	bool success = true;

	/* Call _all_ observers  */
	LOG(DEBUG) << "UpdatesToSceneGraphListener::addGroup: " << assignedId << " to parent " << parentId << ", " << forcedId;
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->addGroup(parentId, assignedId, attributes, forcedId||forcedIdPolicy);
	}
	return success;
}

bool UpdatesToSceneGraphListener::addTransformNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp, bool forcedId) {
	bool success = true;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->addTransformNode(parentId, assignedId, attributes, transform, timeStamp, forcedId||forcedIdPolicy);
	}

	return success;
}

bool UpdatesToSceneGraphListener::addUncertainTransformNode(Id parentId,
		Id& assignedId, vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp, bool forcedId) {
	bool success = true;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->addUncertainTransformNode(parentId, assignedId, attributes, transform, uncertainty, timeStamp, forcedId||forcedIdPolicy);
	}

	return success;
}


bool UpdatesToSceneGraphListener::addGeometricNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, Shape::ShapePtr shape,
		TimeStamp timeStamp, bool forcedId) {
	bool success = true;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->addGeometricNode(parentId, assignedId, attributes, shape, timeStamp, forcedId||forcedIdPolicy);
	}
	return success;
}

bool UpdatesToSceneGraphListener::addRemoteRootNode(Id rootId, vector<Attribute> attributes) {
	bool success = true;

	/* Call _all_ observers  */
	LOG(DEBUG) << "UpdatesToSceneGraphListener::addRemoteRootNode with ID: " << rootId;
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->addRemoteRootNode(rootId,  attributes);
	}
	return success;
}

bool UpdatesToSceneGraphListener::addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId) {
	bool success = true;

	/* Call _all_ observers  */
	LOG(DEBUG) << "UpdatesToSceneGraphListener::addConnection with ID: " << assignedId;
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->addConnection(parentId, assignedId, attributes, sourceIds, targetIds, start, end, forcedId||forcedIdPolicy);
	}
	return success;
}

bool UpdatesToSceneGraphListener::setNodeAttributes(Id id,
		vector<Attribute> newAttributes, TimeStamp timeStamp) {
	bool success = true;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->setNodeAttributes(id, newAttributes, timeStamp);
	}
	return success;
}

bool UpdatesToSceneGraphListener::setTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp) {
	bool success = true;

	/* Inform related observer(s) */
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->setTransform(id, transform, timeStamp);
	}
	return success;
}

bool UpdatesToSceneGraphListener::setUncertainTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp) {
	bool success = true;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->setUncertainTransform(id, transform, uncertainty, timeStamp);
	}
	return success;
}

bool UpdatesToSceneGraphListener::deleteNode(Id id) {
	bool success = true;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->deleteNode(id);
	}
	return success;
}

bool UpdatesToSceneGraphListener::addParent(Id id, Id parentId) {
	bool success = true;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->addParent(id, parentId);
	}
	return success;
}

bool UpdatesToSceneGraphListener::removeParent(Id id, Id parentId) {
	bool success = true;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdate*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		success &= (*observerIterator)->removeParent(id, parentId);
	}
	return success;
}

bool UpdatesToSceneGraphListener::attachSceneGraph(
		ISceneGraphUpdate* observer) {
	assert(observer != 0);
	updateObservers.push_back(observer);
	return true;
}

bool UpdatesToSceneGraphListener::detachSceneGraph(
		ISceneGraphUpdate* observer) {

	assert(observer != 0);
	std::vector<ISceneGraphUpdate*>::iterator observerIterator = std::find(updateObservers.begin(), updateObservers.end(), observer);
    if (observerIterator!=updateObservers.end()) {
    	updateObservers.erase(observerIterator);
    	return true;
    }
    LOG(ERROR) << "Cannot detach update observer. Provided reference does not match with any in the observers list.";
	return false;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
