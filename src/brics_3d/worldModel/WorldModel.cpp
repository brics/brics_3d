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

#include "WorldModel.h"
#include "brics_3d/core/Logger.h"
#include "brics_3d/core/HomogeneousMatrix44.h"

#ifdef BRICS_MICROBLX_ENABLE
#include "ubx.h"
#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#endif

using namespace brics_3d::rsg;

namespace brics_3d {

WorldModel::WorldModel() {

}

WorldModel::~WorldModel() {

}

void WorldModel::getSceneObjects(vector<rsg::Attribute> attributes, vector<SceneObject>& results) {
	TimeStamp currentTime(timer.getCurrentTime(), Units::MilliSecond);
	vector<rsg::Id>resultIds;
	results.clear();

	scene.getNodes(attributes, resultIds);
	for (unsigned int i = 0; i < static_cast<unsigned int>(resultIds.size()); ++i) {
		SceneObject tmpSceneObject;
		tmpSceneObject.id = resultIds[i];

		vector<rsg::Id> parentIds;
		scene.getNodeParents(resultIds[i], parentIds);
		if (static_cast<unsigned int>(parentIds.size()) < 1) {
			tmpSceneObject.parentId = 0; // not initialized
			LOG(WARNING) << "SceneObject has not parent node. Setting value to 0.";
		} else {
			tmpSceneObject.parentId = parentIds[0]; // here we arbitrarily select the fist parent; assumption: tree
		}

		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr tmpTransform(new HomogeneousMatrix44());
		if(!scene.getTransform(resultIds[i], currentTime, tmpTransform)) {
			LOG(WARNING) << "SceneObject with ID " << resultIds[i] << " is not a TansformNode. Skipping.";
			continue;
		}
		tmpSceneObject.transform = tmpTransform;

		vector<rsg::Id> childIds;
		scene.getGroupChildren(resultIds[i], childIds);
		for (unsigned int j = 0; j < static_cast<unsigned int>(childIds.size()); ++j) {
			Shape::ShapePtr tmpShape;
			if (scene.getGeometry(childIds[j], tmpShape, currentTime) == true) {
				tmpSceneObject.shape = tmpShape;
				break; //stop on first found geometry
			}
		}


		vector<rsg::Attribute> tmpAttributes;
		scene.getNodeAttributes(resultIds[i], tmpAttributes);
		tmpSceneObject.attributes = tmpAttributes; //TODO combined attributes from multible nodes?

		results.push_back(tmpSceneObject);
	}
}

void WorldModel::getCurrentTransform(rsg::Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform) {

}

void WorldModel::insertTransform(rsg::Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform) {
	TimeStamp currentTime(timer.getCurrentTime(), Units::MilliSecond);
	scene.setTransform(id, transform, currentTime);
}

void WorldModel::addSceneObject(SceneObject newObject, rsg::Id& assignedId) {
	TimeStamp currentTime(timer.getCurrentTime(), Units::MilliSecond);
	rsg::Id dummyResultID;
	vector<Attribute> emptyAttributes;
	emptyAttributes.clear();

	/* a scene object is essentially a transform */
	scene.addTransformNode(scene.getRootId(), assignedId, newObject.attributes, newObject.transform, currentTime);

	/* add shape as a child node */
	scene.addGeometricNode(assignedId, dummyResultID, emptyAttributes, newObject.shape, currentTime);
}

void WorldModel::initPerception() {

}

void WorldModel::runPerception() {

}

void WorldModel::runOncePerception() {
//	OutdatedDataDeleter* deleter = new OutdatedDataDeleter();
	OutdatedDataIdAwareDeleter* deleter = new OutdatedDataIdAwareDeleter(&scene);
	scene.executeGraphTraverser(deleter, scene.getRootId());
}

void WorldModel::stopPerception() {

}

bool WorldModel::loadFunctionBlock(std::string name) {
#ifdef BRICS_MICROBLX_ENABLE
	LOG(INFO) << "WorldModel::loadFunctionBlock: Loading a new function block to the world model with name " << name;
	/* In a nutshell: you'll need to dlopen(1) the
	 * block or type library and register it with a node, the create the block
	 * instances, configure and start them.
	 */

	/* load dll */
	void* functionBlockHandle;
	functionBlockHandle = dlopen(name.c_str(), RTLD_LAZY);
	if (!functionBlockHandle) {
		LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot load shared library for the function block: " << dlerror();
		return false;
	}

	/* retieve microblock from dll */
	ubx_block_t* block = new ubx_block_t();

	/* init microblx */
	ubx_node_info_t* microBlxNodeHandle = new ubx_node_info_t(); // holds all microblx
	std::string microBlxNodeName = "functionBlocks";

	if (ubx_node_init(microBlxNodeHandle, microBlxNodeName.c_str()) != 0 ) {
		LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot initialize the Microblx node handle.";
		return false;
	}

	/* register function block to microblx "node" */
	if (ubx_block_register(microBlxNodeHandle, block) != 0 ) {
		LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot register the function block to the microblx node handle.";
		return false;
	}

	LOG(DEBUG) << "WorldModel::loadFunctionBlock: function block " << name << " is initialized.";
	return true;

#endif
	LOG(ERROR) << "Microblx support not enabled. Cannot load a function block.";
	return false;

}

bool WorldModel::executeFunctionBlock(std::string name, std::vector<rsg::Id>& input, std::vector<rsg::Id>& output) {
#ifdef BRICS_MICROBLX_ENABLE
	LOG(ERROR) << "Microblx support not enabled. Cannot load a function block.";
#endif
	return false;
}

bool WorldModel::getloadedFunctionBlocks(std::vector<std::string>& functionBlocks) {
#ifdef BRICS_MICROBLX_ENABLE
	LOG(ERROR) << "Microblx support not enabled. Cannot load a function block.";
#endif
	return false;
}

rsg::Id WorldModel::getRootNodeId() {
	return scene.getRootId();
}

brics_3d::rsg::TimeStamp WorldModel::now() {
	return TimeStamp(timer.getCurrentTime(), Units::MilliSecond);
}

} // namespace brics_3d

/* EOF */

