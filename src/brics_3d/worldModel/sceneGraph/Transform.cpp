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

#include "Transform.h"

/* for transform tools: */
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/Logger.h"
#include "PathCollector.h"

namespace brics_3d {

namespace rsg {

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getGlobalTransformAlongPath(Node::NodePath nodePath, TimeStamp timeStamp, TimeStamp& latestStampAlongPath){
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr result(new HomogeneousMatrix44()); //identity matrix
	latestStampAlongPath = TimeStamp(0);
	TimeStamp currentTimeStamp = TimeStamp(0);

	for (unsigned int i = 0; i < static_cast<unsigned int>(nodePath.size()); ++i) {
		Transform* tmpTransform = dynamic_cast<Transform*>(nodePath[i]);
		if (tmpTransform) {
			*result = *( (*result) * (*tmpTransform->getTransform(timeStamp, currentTimeStamp)) );
			if(currentTimeStamp >= latestStampAlongPath) { // store latest stamp
				latestStampAlongPath = currentTimeStamp;
			}
		}
	}
	return result;
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getGlobalTransform(Node::NodePtr node, TimeStamp timeStamp, TimeStamp& actualTimeStamp) {
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr result(new HomogeneousMatrix44()); //identity matrix
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr accumulatedTransform;
	actualTimeStamp = TimeStamp(0); // The latest stamp will be memorized

	/* accumulate parent paths and take the _first_ found path  */
	PathCollector* pathCollector = new PathCollector();
	node->accept(pathCollector);
	if (static_cast<unsigned int>(pathCollector->getNodePaths().size()) > 0) { // != root
		//*result = *((*result) * (*(getGlobalTransformAlongPath(pathCollector->getNodePaths()[0], timeStamp))));
		*result = *((*result) * (*(getGlobalTransformAlongPath(pathCollector->getNodePaths().back(), timeStamp, actualTimeStamp))));
		if (static_cast<unsigned int>(pathCollector->getNodePaths().size()) > 1) {
			LOG(WARNING) << "Multiple transform paths to this node detected. Taking last path and ignoring the rest.";
		}
	}

	/* check if node is a transform on its own ... */
	Transform::TransformPtr tmpTransform = boost::dynamic_pointer_cast<Transform>(node);
	if (tmpTransform) {
		 TimeStamp currentTimeStamp;
		*result = *( (*result) * (*tmpTransform->getTransform(timeStamp, currentTimeStamp)) );
		if(currentTimeStamp >= actualTimeStamp) {
			actualTimeStamp = currentTimeStamp;
		}
	}

	delete pathCollector;
	return result;
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getTransformBetweenNodes(Node::NodePtr node, Node::NodePtr referenceNode, TimeStamp timeStamp, TimeStamp& actualTimeStamp) {
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr result(new HomogeneousMatrix44()); //identity matrix
	TimeStamp latestStampAlongPath1;
	TimeStamp latestStampAlongPath2;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr rootToNodeTransform = getGlobalTransform(node, timeStamp, latestStampAlongPath1);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr rootToReferenceNodeTransform = getGlobalTransform(referenceNode, timeStamp, latestStampAlongPath2);

	rootToReferenceNodeTransform->inverse();
	*result = *( (*rootToReferenceNodeTransform) * (*rootToNodeTransform) ); //cf. Craig p39

	if(latestStampAlongPath1 >= latestStampAlongPath2) {
		actualTimeStamp = latestStampAlongPath1;
	} else {
		actualTimeStamp = latestStampAlongPath2;
	}


	return result;
}


Transform::Transform(TimeStamp maxHistoryDuration) {
	history.clear();
	history.setMaxHistoryDuration(maxHistoryDuration);
	LOG(DEBUG) << "Creating a new Transform with maximal history duration of " << maxHistoryDuration.getSeconds() << "[s].";
	updateCount = 0;
}

Transform::~Transform() {
	history.clear();;
}

bool Transform::insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, TimeStamp timeStamp) {
	assert(newTransform != 0);
	bool success = history.insertData(newTransform, timeStamp);
	if (success) {
		updateCount ++;
		return true;
	}
	return false;
}

void Transform::deleteOutdatedTransforms(TimeStamp latestTimeStamp) {
	history.deleteOutdatedData(latestTimeStamp);
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr  Transform::getTransform(TimeStamp timeStamp) {
	return history.getData(timeStamp);
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr Transform::getTransform(TimeStamp timeStamp, TimeStamp& actualTimeStamp) {
	return history.getData(timeStamp, actualTimeStamp);
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr Transform::getLatestTransform(){
	return history.getData(history.getLatestTimeStamp());
}

TimeStamp Transform::getMaxHistoryDuration() {
	return history.getMaxHistoryDuration();
}

void Transform::setMaxHistoryDuration(TimeStamp maxHistoryDuration) {
	history.setMaxHistoryDuration(maxHistoryDuration);
}

unsigned int Transform::getCurrentHistoryLenght() {
	return history.size();
}

TimeStamp Transform::getLatestTimeStamp() {
	return history.getLatestTimeStamp();
}

TimeStamp Transform::getOldestTimeStamp() {
	return history.getOldestTimeStamp();
}

unsigned int Transform::getUpdateCount() {
    return updateCount;
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

TemporalCache<IHomogeneousMatrix44::IHomogeneousMatrix44Ptr> Transform::getHistory() {
	return history;
}


std::string Transform::cacheToString() {
	return history.cacheToString();
}

} // namespace brics_3d::RSG

} // namespace brics_3d

/* EOF */

