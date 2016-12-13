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

#include "TimeStamper.h"
#include <brics_3d/core/Logger.h>

#include <iomanip> // setprecision

using brics_3d::Logger;

namespace brics_3d {
namespace rsg {

TimeStamper::TimeStamper(WorldModel* wmHandle, std::string name) : wm(wmHandle) {
	timeStampLog = new Benchmark(name);
	timeStampLog->output << "#timeStamp COMMAND UUID" << std::endl;
	timeStampLog->output << "# ADD_NODE has a value of " << ADD_NODE << std::endl;
	timeStampLog->output << "# ADD_GROUP has a value of " << ADD_GROUP << std::endl;
	timeStampLog->output << "# ADD_TRANSFORM_NODE has a value of " << ADD_TRANSFORM_NODE << std::endl;
	timeStampLog->output << "# ADD_UNCERTAIN_TRANSFORM_NODE has a value of " << ADD_UNCERTAIN_TRANSFORM_NODE << std::endl;
	timeStampLog->output << "# ADD_GEOMETRIC_NODE has a value of " << ADD_GEOMETRIC_NODE << std::endl;
	timeStampLog->output << "# SET_NODE_ATTRIBUTES has a value of" << SET_NODE_ATTRIBUTES << std::endl;
	timeStampLog->output << "# SET_TRANSFORM has a value of " << SET_TRANSFORM << std::endl;
	timeStampLog->output << "# SET_UNCERTAIN_TRANSFORM has a value of " << SET_UNCERTAIN_TRANSFORM << std::endl;
	timeStampLog->output << "# DELETE_NODE has a value of " << DELETE_NODE << std::endl;
	timeStampLog->output << "# REMOVE_PARENT has a value of " << REMOVE_PARENT << std::endl;

}

TimeStamper::~TimeStamper() {
	delete timeStampLog;
}

bool TimeStamper::addNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << ADD_NODE << " " << assignedId << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addNode(parentId, assignedId, attributes, forcedId);
	}
	return true;
}

bool TimeStamper::addGroup(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << ADD_GROUP << " " << assignedId << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addGroup(parentId, assignedId, attributes, forcedId);
	}
	return true;
}

bool TimeStamper::addTransformNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp, bool forcedId) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << ADD_TRANSFORM_NODE << " " << assignedId << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addTransformNode(parentId, assignedId, attributes, transform, timeStamp, forcedId);
	}
	return true;
}

bool TimeStamper::addUncertainTransformNode(Id parentId,
		Id& assignedId, vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp, bool forcedId) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << ADD_UNCERTAIN_TRANSFORM_NODE << " " << assignedId << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addUncertainTransformNode(parentId, assignedId, attributes, transform, uncertainty, timeStamp, forcedId);
	}
	return true;
}

bool TimeStamper::addGeometricNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, Shape::ShapePtr shape,
		TimeStamp timeStamp, bool forcedId) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << ADD_GEOMETRIC_NODE << " " << assignedId << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addGeometricNode(parentId, assignedId, attributes, shape, timeStamp, forcedId);
	}
	return true;
}

bool TimeStamper::addRemoteRootNode(Id rootId, vector<Attribute> attributes) {
	return false;
}

bool TimeStamper::addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId) {
	return false;
}


bool TimeStamper::setNodeAttributes(Id id,
		vector<Attribute> newAttributes, TimeStamp timeStamp) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << SET_NODE_ATTRIBUTES << " " << id << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->setNodeAttributes(id, newAttributes, timeStamp);
	}
	return true;
}

bool TimeStamper::setTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << SET_TRANSFORM << " " << id << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->setTransform(id, transform, timeStamp);
	}
	return true;
}

bool TimeStamper::setUncertainTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << SET_UNCERTAIN_TRANSFORM << " " << id << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->setUncertainTransform(id, transform, uncertainty, timeStamp);
	}
	return true;
}

bool TimeStamper::deleteNode(Id id) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << DELETE_NODE << " " << id << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->deleteNode(id);
	}
	return true;
}

bool TimeStamper::addParent(Id id, Id parentId) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << ADD_PARENT << " " << id << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addParent(id, parentId);
	}
	return true;
}

bool TimeStamper::removeParent(Id id, Id parentId) {

	timeStampLog->output << std::setprecision(15) << wm->now().getSeconds() << " " << REMOVE_PARENT << " " << id << std::endl;

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->removeParent(id, parentId);
	}
	return true;
}

bool TimeStamper::attachUpdateObserver(
		ISceneGraphUpdateObserver* observer) {
	assert(observer != 0);
	updateObservers.push_back(observer);
	return true;
}

bool TimeStamper::detachUpdateObserver(
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

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
