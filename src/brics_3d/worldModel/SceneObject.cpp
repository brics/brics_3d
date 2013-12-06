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

#include "SceneObject.h"
#include "sceneGraph/GeometricNode.h"
#include "sceneGraph/Attribute.h"

namespace brics_3d {

SceneObject::SceneObject() {
  // Bouml preserved body begin 0002D283
  // Bouml preserved body end 0002D283
}

SceneObject::~SceneObject() {
  // Bouml preserved body begin 0002D303
  // Bouml preserved body end 0002D303
}

SceneObject::SceneObject(SceneObject & source) {
	this->id = source.id;
	this->parentId = source.parentId;
    this->transform = source.transform;
    this->shape = source.shape;
    this->attributes = source.attributes;
}

SceneObject::SceneObject(const SceneObject & source) {
	this->id = source.id;
	this->parentId = source.parentId;
    this->transform = source.transform;
    this->shape = source.shape;
    this->attributes = source.attributes;
}

SceneObject & SceneObject::operator=(SceneObject & source) {
	this->id = source.id;
	this->parentId = source.parentId;
    this->transform = source.transform;
    this->shape = source.shape;
    this->attributes = source.attributes;
    return *this;
}

SceneObject & SceneObject::operator=(const SceneObject & source) {
	this->id = source.id;
	this->parentId = source.parentId;
    this->transform = source.transform;
    this->shape = source.shape;
    this->attributes = source.attributes;
    return *this;
}

ostream & operator<<(ostream & os, const SceneObject & x) {

	os << "id: " << x.id << std::endl;
	os << "parentId: " << x.parentId << std::endl;
	os << "transform: " << x.transform << std::endl;
	os << "transform: " << x.transform << std::endl;
	os << "attributes: ";
	for (unsigned int i = 0; i < x.attributes.size(); ++i) {
		os <<  x.attributes[i];
	}
	os << std::endl;

	return os;
}


} // namespace brics_3d

/* EOF */

