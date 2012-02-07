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
#include "core/HomogeneousMatrix44.h"
#include "core/Logger.h"
#include "PathCollector.h"

//#define STATIC_TRANSFORM

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

Transform::Transform() : maxHistoryDuration(dafaultMaxHistoryDuration), updateCount(0) {
#ifdef STATIC_TRANSFORM
	history.resize(1);
#else
	history.clear();
#endif
}

Transform::~Transform() {
#ifdef STATIC_TRANSFORM
	history.clear();
#else
	history.clear();
#endif
}

void Transform::insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, TimeStamp timeStamp) {
	assert(newTransform != 0);
#ifdef STATIC_TRANSFORM
	history[0].first = newTransform;
	history[0].second = timeStamp;
#else

	/* history policy: descending order of timestamps (the older the closer to the end - like humans...)
	 *  latest              oldest
	 *   |-------------------|
	 *  begin               end
	 */

	historyIterator = history.begin();

	/* insert new data at its correct place in time */
	while(historyIterator != history.end()) { // loop over history
		if (historyIterator->second <= timeStamp) {
			break;
		}
		historyIterator++;
	}
	history.insert(historyIterator, std::make_pair(newTransform, timeStamp)); // fit into correct temporal place


	/*
	 * in this case the temporal reference is deduced from the stored data and not
	 * from the current (real) time.
	 */
	historyIterator = history.begin(); // we already know that there is already one element...
	TimeStamp latestTimeStamp = history.begin()->second;
	deleteOutdatedTransforms(latestTimeStamp);
	updateCount ++;
#endif
}

void Transform::deleteOutdatedTransforms(TimeStamp latestTimeStamp) {
	/*
	 * delete all transforms where the durartion (delta between latestTime and stored) exeeds
	 * the defined maximum history duration
	 */
	while(!history.empty() && (history.back().second + maxHistoryDuration < latestTimeStamp)) {
		history.pop_back();
	}
}

HistoryIterator Transform::getClosestTransform(TimeStamp timeStamp) {
	HistoryIterator resultIterator;
	HistoryIterator previousIterator; //remember: values have a decending order
	resultIterator = history.begin();

	if(getCurrentHistoryLenght() == 1) { // special case for first element -> just return it
		return resultIterator;
	}

	while(resultIterator != history.end()) { // loop over history
		if (timeStamp >= resultIterator->second) {
			if(resultIterator != history.begin()) { // i.e. a previous element exists => copare wich is actually the closest
				previousIterator = resultIterator - 1;
				if ( (previousIterator->second - timeStamp) <= (timeStamp - resultIterator->second) ) {
					return previousIterator;
				} else {
					return resultIterator;
				}
			} else {
				return resultIterator;
			}
		}
		resultIterator++;
	}

	/*
	 * We might reach this line when timeStamp is older thant the oldest element in the history.
	 * In that case we want to return the last/oldest element.
	 */
	if(getCurrentHistoryLenght() > 1) {
		resultIterator = history.end() - 1;
		assert(timeStamp < resultIterator->second); // just to be sure...
	}
	return resultIterator;
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr  Transform::getTransform(TimeStamp timeStamp) {
#ifdef STATIC_TRANSFORM
	return history[0].first;
#else
	HistoryIterator closestTransform = getClosestTransform(timeStamp);
	if(closestTransform == history.end()) {
		LOG(WARNING) << "Transform history for node " << this->getId() << " is empty. Cannot find a transform for time stamp ";// << timeStamp;
		return IHomogeneousMatrix44::IHomogeneousMatrix44Ptr(); // should be kind of null...
		// TODO throw exception instead?
	}
	return closestTransform->first;
#endif
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr Transform::getLatestTransform(){
#ifdef STATIC_TRANSFORM
	return history[0].first;
#else
	historyIterator = history.begin();
	if(historyIterator == history.end()) {
		LOG(WARNING) << "Transform history for node " << this->getId() << " is empty. Cannot find latest transform.";// << timeStamp;
		return IHomogeneousMatrix44::IHomogeneousMatrix44Ptr(); // should be kind of null...
		// TODO throw exception instead?
	}
	return historyIterator->first;
#endif
}

TimeStamp Transform::getMaxHistoryDuration() {
    return maxHistoryDuration;
}

void Transform::setMaxHistoryDuration(TimeStamp maxHistoryDuration) {
    this->maxHistoryDuration = maxHistoryDuration;
}

unsigned int Transform::getCurrentHistoryLenght() {
	return static_cast<unsigned int>(history.size());
}

TimeStamp Transform::getLatestTimeStamp() {
	historyIterator = history.begin();
	if(historyIterator == history.end()) {
		LOG(WARNING) << "Transform history for node " << this->getId() << " is empty. Cannot find latest time stamp.";// << timeStamp;
		return TimeStamp(0.0);
	}
	return history.front().second;
}

TimeStamp Transform::getOldestTimeStamp() {
	historyIterator = history.begin();
	if(historyIterator == history.end()) {
		LOG(WARNING) << "Transform history for node " << this->getId() << " is empty. Cannot find oldest time stamp.";// << timeStamp;
		return TimeStamp(0.0);
	}
	return history.back().second;
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

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D

/* EOF */

