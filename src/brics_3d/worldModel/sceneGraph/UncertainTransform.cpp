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

UncertainTransform::UncertainTransform(TimeStamp maxHistoryDuration) {
	uncertaintyHistory.clear();
	this->setMaxHistoryDuration(maxHistoryDuration);
	uncertaintyUpdateCount = 0;
}

UncertainTransform::~UncertainTransform() {
	uncertaintyHistory.clear();
}

void UncertainTransform::insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, ITransformUncertainty::ITransformUncertaintyPtr newUncertainty, TimeStamp timeStamp) {
	assert(newTransform != 0);
	assert(newUncertainty != 0);

	/* update the transform cache (cf. super class) */
	Transform::insertTransform(newTransform, timeStamp);
	uncertaintyHistory.insertData(newUncertainty, timeStamp);
}

void UncertainTransform::insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, TimeStamp timeStamp) {
	/* update the transform cache (cf. super class) */
	Transform::insertTransform(newTransform, timeStamp);
	uncertaintyHistory.deleteOutdatedData(uncertaintyHistory.getLatestTimeStamp());
}

ITransformUncertainty::ITransformUncertaintyPtr UncertainTransform::getTransformUncertainty(TimeStamp timeStamp) {
	return uncertaintyHistory.getData(timeStamp);
}


ITransformUncertainty::ITransformUncertaintyPtr UncertainTransform::getLatestTransformUncertainty() {
	return uncertaintyHistory.getData(uncertaintyHistory.getLatestTimeStamp());
}

TimeStamp UncertainTransform::getLatestTransformUncertaintyTimeStamp() {
	return uncertaintyHistory.getLatestTimeStamp();
}

TimeStamp UncertainTransform::getOldestTransformUncertaintyTimeStamp() {
	return uncertaintyHistory.getOldestTimeStamp();
}

unsigned int UncertainTransform::getCurrentUncertaintyHistoryLength() {
	return uncertaintyHistory.getNumberOfCacheEntries();
}

void UncertainTransform::setMaxHistoryDuration(TimeStamp maxHistoryDuration) {
	// Update parameter for _both_ caches.
	Transform::setMaxHistoryDuration(maxHistoryDuration);
	uncertaintyHistory.setMaxHistoryDuration(maxHistoryDuration);
}

void UncertainTransform::deleteOutdatedTransformUncertainties(TimeStamp latestTimeStamp) {
	uncertaintyHistory.deleteOutdatedData(latestTimeStamp);
}

void UncertainTransform::deleteOutdatedTransforms(TimeStamp latestTimeStamp) {
	Transform::deleteOutdatedTransforms(latestTimeStamp);
	uncertaintyHistory.deleteOutdatedData(latestTimeStamp); //this one is debatable
}

}

}

/* EOF */
