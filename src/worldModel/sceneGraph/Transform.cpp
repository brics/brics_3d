/******************************************************************************
* Copyright (c) 2011
* GPS GmbH
*
* Author:
* Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of GPS GmbH nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include "Transform.h"

/* for transform tools: */
#include "core/HomogeneousMatrix44.h"
#include "core/Logger.h"
#include "PathCollector.h"

namespace BRICS_3D {

namespace RSG {

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getGlobalTransformAlongPath(Node::NodePath nodePath){
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr result(new HomogeneousMatrix44()); //identity matrix
	for (unsigned int i = 0; i < static_cast<unsigned int>(nodePath.size()); ++i) {
		Transform* tmpTransform = dynamic_cast<Transform*>(nodePath[i]);
		if (tmpTransform) {
			*result = *( (*result) * (*tmpTransform->getLatestTransform()) );
		}
	}
	return result;
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getGlobalTransform(Node::NodePtr node) {
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr result(new HomogeneousMatrix44()); //identity matrix
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr accumulatedTransform;

	/* accumulate parent paths and take the _first_ found path  */
	PathCollector* pathCollector = new PathCollector();
	node->accept(pathCollector);
	if (static_cast<unsigned int>(pathCollector->getNodePaths().size()) > 0) { // != root
		*result = *((*result) * (*(getGlobalTransformAlongPath(pathCollector->getNodePaths()[0]))));
		if (static_cast<unsigned int>(pathCollector->getNodePaths().size()) > 1) {
			LOG(WARNING) << "Multiple transform paths to this node detected. Taking fist path and ignoring the rest.";
		}
	}

	/* check if node is a transform on its own ... */
	Transform::TransformPtr tmpTransform = boost::dynamic_pointer_cast<Transform>(node);
	if (tmpTransform) {
		*result = *( (*result) * (*tmpTransform->getLatestTransform()) );
	}

	delete pathCollector;
	return result;
}

Transform::Transform() {
	history.resize(1);
}

Transform::~Transform() {
	history.clear();
}

void Transform::insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, TimeStamp timeStamp) {
	assert(newTransform != 0);
	history[0].first = newTransform;
	history[0].second = timeStamp;
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr  Transform::getTransform(TimeStamp timeStamp) {
	return history[0].first;
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr Transform::getLatestTransform(){
	return history[0].first;
}

void Transform::accept(INodeVisitor* visitor){
	visitor->visit(this);
	if (visitor->getDirection() == INodeVisitor::upwards) { //TODO move to "traverseUpwards" method?
	    for(unsigned i = 0; i < getNumberOfParents(); ++i) // recursively go up the graph structure
	    {
	        getParent(i)->accept(visitor);
	    }
	} else if (visitor->getDirection() == INodeVisitor::downwards) { //TODO move to "traverseDownwards" method?
		for(unsigned i = 0; i < getNumberOfChildren(); ++i) // recursively go down the graph structure
		{
			getChild(i)->accept(visitor);
		}
	}
}

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D

/* EOF */

