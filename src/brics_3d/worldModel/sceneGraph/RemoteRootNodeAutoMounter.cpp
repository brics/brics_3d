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

#include "RemoteRootNodeAutoMounter.h"
#include <brics_3d/core/Logger.h>


using brics_3d::Logger;

namespace brics_3d {
namespace rsg {

RemoteRootNodeAutoMounter::RemoteRootNodeAutoMounter(SceneGraphFacade* observedScene) {
	assert(observedScene != 0);
	this->observedScene = observedScene;
	this->mountPoint = observedScene->getRootId();
}

RemoteRootNodeAutoMounter::RemoteRootNodeAutoMounter(SceneGraphFacade* observedScene, Id mountPoint) {
	assert(observedScene != 0);
	this->observedScene = observedScene;
	this->mountPoint = mountPoint;
}

RemoteRootNodeAutoMounter::~RemoteRootNodeAutoMounter() {

}

bool RemoteRootNodeAutoMounter::addNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {
	return true;
}

bool RemoteRootNodeAutoMounter::addGroup(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {
	return true;
}

bool RemoteRootNodeAutoMounter::addTransformNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp, bool forcedId) {
	return true;
}

bool RemoteRootNodeAutoMounter::addUncertainTransformNode(Id parentId,
		Id& assignedId, vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp, bool forcedId) {
	return true;
}


bool RemoteRootNodeAutoMounter::addGeometricNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, Shape::ShapePtr shape,
		TimeStamp timeStamp, bool forcedId) {
	return true;
}

bool RemoteRootNodeAutoMounter::addRemoteRootNode(Id rootId, vector<Attribute> attributes) {
	LOG(DEBUG) << "RemoteRootNodeAutoMounter::addRemoteRootNode with ID: " << rootId;
	return observedScene->addParent(rootId, mountPoint);
}

bool RemoteRootNodeAutoMounter::setNodeAttributes(Id id,
		vector<Attribute> newAttributes) {
	return true;
}

bool RemoteRootNodeAutoMounter::setTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp) {
	return true;
}

bool RemoteRootNodeAutoMounter::setUncertainTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp) {
	return true;
}

bool RemoteRootNodeAutoMounter::deleteNode(Id id) {
	return true;
}

bool RemoteRootNodeAutoMounter::addParent(Id id, Id parentId) {
	return true;
}

bool RemoteRootNodeAutoMounter::removeParent(Id id, Id parentId) {
	return true;
}



} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
