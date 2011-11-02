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

namespace BRICS_3D {

WorldModel::WorldModel() {

}

WorldModel::~WorldModel() {

}

void WorldModel::getSceneObjects(vector<Attribute> attributes, vector<SceneObject>* results) {

}

void WorldModel::getCurrentTransform(unsigned int id, IHomogeneousMatrix44* transform) {

}


void WorldModel::addSceneObject(SceneObject newObject, unsigned int* assignedId) {

}

void WorldModel::initPerception() {

}

void WorldModel::runPerception() {

}

void WorldModel::runOncePerception() {

}

void WorldModel::stopPerception() {

}

unsigned int WorldModel::getRootNodeId() {
	return 1; //0 might be not initialized
}

} // namespace BRICS_3D

/* EOF */

