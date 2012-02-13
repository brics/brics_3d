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
#include "core/Logger.h"
#include "core/HomogeneousMatrix44.h"

namespace BRICS_3D {

WorldModel::WorldModel() {

}

WorldModel::~WorldModel() {

}

void WorldModel::getSceneObjects(vector<Attribute> attributes, vector<SceneObject>& results) {
	TimeStamp currentTime(timer.getCurrentTime());
	vector<unsigned int>resultIds;
	results.clear();

	scene.getNodes(attributes, resultIds);
	for (unsigned int i = 0; i < static_cast<unsigned int>(resultIds.size()); ++i) {
		SceneObject tmpSceneObject;
		tmpSceneObject.id = resultIds[i];

		vector<unsigned int> parentIds;
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

		vector<unsigned int> childIds;
		scene.getGroupChildren(resultIds[i], childIds);
		for (unsigned int j = 0; j < static_cast<unsigned int>(childIds.size()); ++j) {
			Shape::ShapePtr tmpShape;
			if (scene.getGeometry(childIds[j], tmpShape, currentTime) == true) {
				tmpSceneObject.shape = tmpShape;
				break; //stop on first found geometry
			}
		}


		vector<RSG::Attribute> tmpAttributes;
		scene.getNodeAttributes(resultIds[i], tmpAttributes);
		tmpSceneObject.attributes = tmpAttributes; //TODO combined attributes from multible nodes?

		results.push_back(tmpSceneObject);
	}
}

void WorldModel::getCurrentTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform) {

}

void WorldModel::insertTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform) {
	TimeStamp currentTime(timer.getCurrentTime());
	scene.setTransform(id, transform, currentTime);
}

void WorldModel::addSceneObject(SceneObject newObject, unsigned int& assignedId) {
	TimeStamp currentTime(timer.getCurrentTime());
	unsigned int dummyResultID;
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
	scene.executeGraphTraverser(deleter);
}

void WorldModel::stopPerception() {

}

unsigned int WorldModel::getRootNodeId() {
	return scene.getRootId();
}

} // namespace BRICS_3D

/* EOF */

