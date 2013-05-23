/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2013, KU Leuven
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

#include "UncertainTransform.h"
#include "brics_3d/core/Logger.h"

namespace brics_3d {

namespace rsg {

UncertainTransform::UncertainTransform() {
	uncertaintyHistory.clear();
}

UncertainTransform::~UncertainTransform() {
	uncertaintyHistory.clear();
}

void UncertainTransform::insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, ITransformUncertainty::ITransformUncertaintyPtr newUncertainty, TimeStamp timeStamp) {
	assert(newTransform != 0);
	assert(newUncertainty != 0);

	/* update the transform cache (cf. super class) */
	Transform::insertTransform(newTransform, timeStamp);

	/* update the uncertainty cache */

	/* history policy: descending order of timestamps (the older the closer to the end - like humans...)
	 *  latest              oldest
	 *   |-------------------|
	 *  begin               end
	 */

	uncertaintyHistoryIterator = uncertaintyHistory.begin();

	/* insert new data at its correct place in time */
	while(uncertaintyHistoryIterator != uncertaintyHistory.end()) { // loop over history
		if (uncertaintyHistoryIterator->second <= timeStamp) {
			break;
		}
		uncertaintyHistoryIterator++;
	}
	uncertaintyHistory.insert(uncertaintyHistoryIterator, std::make_pair(newUncertainty, timeStamp)); // fit into correct temporal place


	/*
	 * in this case the temporal reference is deduced from the stored data and not
	 * from the current (real) time.
	 */
	uncertaintyHistoryIterator = uncertaintyHistory.begin(); // we already know that there is already one element...
	TimeStamp latestTimeStamp = uncertaintyHistory.begin()->second;
	deleteOutdatedTransformUncertainties(latestTimeStamp); //deletion for Transform data is already handeled within Transform::insertTransform

	//NOTE: update count is already incerased in Transform::insertTransform, so we skip it here.

}

void UncertainTransform::insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, TimeStamp timeStamp) {
	Transform::insertTransform(newTransform, timeStamp);

	/*
	 * in this case the temporal reference is deduced from the stored data and not
	 * from the current (real) time.
	 */
	if(uncertaintyHistoryIterator !=  uncertaintyHistory.end()) {
		uncertaintyHistoryIterator = uncertaintyHistory.begin();
		TimeStamp latestTimeStamp = uncertaintyHistory.begin()->second;
		deleteOutdatedTransformUncertainties(latestTimeStamp);
	}

}

ITransformUncertainty::ITransformUncertaintyPtr UncertainTransform::getTransformUncertainty(TimeStamp timeStamp) {
	UncertaintyHistoryIterator closestTransformUncertainty = getClosestTransformUncertainty(timeStamp);
	if(closestTransformUncertainty == uncertaintyHistory.end()) {
		LOG(WARNING) << "Transform uncertainty history for node " << this->getId() << " is empty. Cannot find a transform uncertainty for time stamp ";
		return ITransformUncertainty::ITransformUncertaintyPtr(); // should be kind of null...
	}
	return closestTransformUncertainty->first;
}


ITransformUncertainty::ITransformUncertaintyPtr UncertainTransform::getLatestTransformUncertainty() {
	uncertaintyHistoryIterator = uncertaintyHistory.begin();
	if(uncertaintyHistoryIterator == uncertaintyHistory.end()) {
		LOG(WARNING) << "Transform uncertainty history for node " << this->getId() << " is empty. Cannot find latest transform uncertainty.";
		return ITransformUncertainty::ITransformUncertaintyPtr(); // should be kind of null...
	}
	return uncertaintyHistoryIterator->first;
}

TimeStamp UncertainTransform::getLatestTransformUncertaintyTimeStamp() {
	uncertaintyHistoryIterator = uncertaintyHistory.begin();
	if(uncertaintyHistoryIterator == uncertaintyHistory.end()) {
		LOG(WARNING) << "Transform uncertainty history for node " << this->getId() << " is empty. Cannot find latest time stamp.";// << timeStamp;
		return TimeStamp(0.0);
	}
	return uncertaintyHistory.front().second;
}

TimeStamp UncertainTransform::getOldestTransformUncertaintyTimeStamp() {
	uncertaintyHistoryIterator = uncertaintyHistory.begin();
	if(uncertaintyHistoryIterator == uncertaintyHistory.end()) {
		LOG(WARNING) << "Transform uncertainty history for node " << this->getId() << " is empty. Cannot find latest time stamp.";// << timeStamp;
		return TimeStamp(0.0);
	}
	return uncertaintyHistory.back().second;
}

unsigned int UncertainTransform::getCurrentUncertaintyHistoryLength() {
	return static_cast<unsigned int>(uncertaintyHistory.size());
}

void UncertainTransform::deleteOutdatedTransformUncertainties(TimeStamp latestTimeStamp) {
	/*
	 * delete all transforms where the durartion (delta between latestTime and stored) exeeds
	 * the defined maximum history duration
	 */
	while(!uncertaintyHistory.empty() && (uncertaintyHistory.back().second + maxHistoryDuration < latestTimeStamp)) {
		uncertaintyHistory.pop_back();
	}
}

void UncertainTransform::deleteOutdatedTransforms(TimeStamp latestTimeStamp) {
	Transform::deleteOutdatedTransforms(latestTimeStamp);
	deleteOutdatedTransformUncertainties(latestTimeStamp); //this one is debatable
}

UncertaintyHistoryIterator UncertainTransform::getClosestTransformUncertainty(TimeStamp timeStamp) {
	//TODO: turn the temporal cache into a class of its own
	UncertaintyHistoryIterator resultIterator;
	UncertaintyHistoryIterator previousIterator; //remember: values have a decending order
	resultIterator = uncertaintyHistory.begin();

	if(getCurrentUncertaintyHistoryLength() == 1) { // special case for first element -> just return it
		return resultIterator;
	}

	while(resultIterator != uncertaintyHistory.end()) { // loop over history
		if (timeStamp >= resultIterator->second) {
			if(resultIterator != uncertaintyHistory.begin()) { // i.e. a previous element exists => copare wich is actually the closest
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
	 * We might reach this line when timeStamp is older that the oldest element in the history.
	 * In that case we want to return the last/oldest element.
	 */
	if(getCurrentUncertaintyHistoryLength() > 1) {
		resultIterator = uncertaintyHistory.end() - 1;
		assert(timeStamp < resultIterator->second); // just to be sure...
	}
	return resultIterator;

}

}

}

/* EOF */
