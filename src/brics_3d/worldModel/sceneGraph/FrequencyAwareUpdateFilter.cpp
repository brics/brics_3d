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

#include "FrequencyAwareUpdateFilter.h"
#include <brics_3d/core/Logger.h>


using brics_3d::Logger;

namespace brics_3d {
namespace rsg {

FrequencyAwareUpdateFilter::FrequencyAwareUpdateFilter() {
	lastTransformUpdate = TimeStamp(0, Units::Second);
	lastGeometricNodeUpdate= TimeStamp(0, Units::Second);
}

FrequencyAwareUpdateFilter::~FrequencyAwareUpdateFilter() {

}

bool FrequencyAwareUpdateFilter::addNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addNode(parentId, assignedId, attributes, forcedId);
	}
	return true;
}

bool FrequencyAwareUpdateFilter::addGroup(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addGroup(parentId, assignedId, attributes, forcedId);
	}
	return true;
}

bool FrequencyAwareUpdateFilter::addTransformNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp, bool forcedId) {

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addTransformNode(parentId, assignedId, attributes, transform, timeStamp, forcedId);
	}

	return true;

}

bool FrequencyAwareUpdateFilter::addUncertainTransformNode(Id parentId,
		Id& assignedId, vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp, bool forcedId) {

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addUncertainTransformNode(parentId, assignedId, attributes, transform, uncertainty, timeStamp, forcedId);
	}

	return true;
}


bool FrequencyAwareUpdateFilter::addGeometricNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, Shape::ShapePtr shape,
		TimeStamp timeStamp, bool forcedId) {

	TimeStamp currentTime(timer.getCurrentTime(), brics_3d::Units::MilliSecond);
	Duration durationSinceLastInvocation = currentTime - lastGeometricNodeUpdate;

	if (durationSinceLastInvocation > maxGeometricNodeUpdateDuration) {

		/* Inform related observer(s) */
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			(*observerIterator)->addGeometricNode(parentId, assignedId, attributes, shape, timeStamp, forcedId);
		}

		lastGeometricNodeUpdate = currentTime;
		return true;

	} else {
		LOG(DEBUG) << "FrequencyAwareUpdateFilter:GeometricNode update is skipped due to a duration since last invokation of " <<
				durationSinceLastInvocation.getSeconds() << " that is greater than the allowed maximum of " << maxGeometricNodeUpdateDuration.getSeconds() << ".";
	}

	return false;
}

bool FrequencyAwareUpdateFilter::setNodeAttributes(Id id,
		vector<Attribute> newAttributes) {

	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->setNodeAttributes(id, newAttributes);
	}
	return true;
}

bool FrequencyAwareUpdateFilter::setTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp) {


	TimeStamp currentTime(timer.getCurrentTime(), brics_3d::Units::MilliSecond);
	Duration durationSinceLastInvocation = currentTime - lastTransformUpdate;


	if (durationSinceLastInvocation > maxTransformUpdateDuration) {

		/* Inform related observer(s) */
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			(*observerIterator)->setTransform(id, transform, timeStamp);
		}

		lastTransformUpdate = currentTime;
		return true;

	} else {
		LOG(DEBUG) << "FrequencyAwareUpdateFilter: setTransform update is skipped due to a duration since last invokation of " <<
				durationSinceLastInvocation.getSeconds() << " that is greater than the allowed maximum of " << maxGeometricNodeUpdateDuration.getSeconds() << ".";
	}

	return false;
}

bool FrequencyAwareUpdateFilter::setUncertainTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp) {


	TimeStamp currentTime(timer.getCurrentTime(), brics_3d::Units::MilliSecond);
	Duration durationSinceLastInvocation = currentTime - lastTransformUpdate;


	if (durationSinceLastInvocation > maxTransformUpdateDuration) {

		/* Inform related observer(s) */
		std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
		for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
			(*observerIterator)->setUncertainTransform(id, transform, uncertainty, timeStamp);
		}

		lastTransformUpdate = currentTime;
		return true;

	} else {
		LOG(DEBUG) << "FrequencyAwareUpdateFilter: setUncertainTransform update is skipped due to a duration since last invokation of " <<
				durationSinceLastInvocation.getSeconds() << " that is greater than the allowed maximum of " << maxGeometricNodeUpdateDuration.getSeconds() << ".";
	}

	return false;
}

bool FrequencyAwareUpdateFilter::deleteNode(Id id) {
	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->deleteNode(id);
	}
	return true;
}

bool FrequencyAwareUpdateFilter::addParent(Id id, Id parentId) {
	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->addParent(id, parentId);
	}
	return true;
}

bool FrequencyAwareUpdateFilter::removeParent(Id id, Id parentId) {
	/* Call _all_ observers  */
	std::vector<ISceneGraphUpdateObserver*>::iterator observerIterator;
	for (observerIterator = updateObservers.begin(); observerIterator != updateObservers.end(); ++observerIterator) {
		(*observerIterator)->removeParent(id, parentId);
	}
	return true;
}

bool FrequencyAwareUpdateFilter::attachUpdateObserver(
		ISceneGraphUpdateObserver* observer) {
	assert(observer != 0);
	updateObservers.push_back(observer);
	return true;
}

bool FrequencyAwareUpdateFilter::detachUpdateObserver(
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
