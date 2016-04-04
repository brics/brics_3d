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
	this->createMountPointIfItDoesNotExist = false;
}

RemoteRootNodeAutoMounter::RemoteRootNodeAutoMounter(SceneGraphFacade* observedScene, Id mountPoint, bool createMountPointIfItDoesNotExist) {
	assert(observedScene != 0);
	this->observedScene = observedScene;
	this->mountPoint = mountPoint;
	this->createMountPointIfItDoesNotExist = createMountPointIfItDoesNotExist;
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
	if(this->createMountPointIfItDoesNotExist) {
		LOG(DEBUG) << "RemoteRootNodeAutoMounter::addRemoteRootNode createMountPointIfItDoesNotExist is activated.";
		vector<Attribute> tmpAttributes;
		bool mountPointExists = observedScene->getNodeAttributes(mountPoint, tmpAttributes);
		if(!mountPointExists) {
			LOG(DEBUG) << "RemoteRootNodeAutoMounter::addRemoteRootNode mountPoint does not exist. Creating a new one as remote root.";
			tmpAttributes.clear();
			tmpAttributes.push_back(Attribute("rsg::dbg","added_by_auto_mounter"));
			observedScene->addRemoteRootNode(mountPoint, tmpAttributes);
		}
	}

	if(rootId == mountPoint) {
		LOG(DEBUG) << "RemoteRootNodeAutoMounter::addRemoteRootNode rootId and mountPoint are identical, so they are skipped. ( = " << rootId  << ")";
		return false;
	}

	return observedScene->addParent(rootId, mountPoint);
}

bool RemoteRootNodeAutoMounter::addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId) {
	return true;
}


bool RemoteRootNodeAutoMounter::setNodeAttributes(Id id,
		vector<Attribute> newAttributes, TimeStamp timeStamp) {
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
