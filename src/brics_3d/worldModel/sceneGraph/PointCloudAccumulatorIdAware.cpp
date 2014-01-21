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


#include "PointCloudAccumulatorIdAware.h"
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/Logger.h"

using brics_3d::Logger;

namespace brics_3d {

namespace rsg {

PointCloudAccumulatorIdAware::PointCloudAccumulatorIdAware(SceneGraphFacade* facadeHandle, Id referenceNodeId) : PointCloudAccumulator(Node::NodePtr()) {
	assert(facadeHandle != 0);
	this->facadeHandle = facadeHandle;
	this->referenceNodeId = referenceNodeId;
}

PointCloudAccumulatorIdAware::~PointCloudAccumulatorIdAware() {
	facadeHandle = 0; //we do not delete, as we are not the owner
}

IHomogeneousMatrix44::IHomogeneousMatrix44Ptr PointCloudAccumulatorIdAware::doGetTransformFromReferenceToPointCloud(GeometricNode* node) {
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformFromReferenceToPointCloud;

	if(!facadeHandle->getTransformForNode(node->getId(), referenceNodeId, TimeStamp(), transformFromReferenceToPointCloud)) {
		LOG(ERROR) << "PointCloudAccumulatorIdAware: Cannot find appropriate transfrom. Returning identity.";

		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identity(new HomogeneousMatrix44());
		transformFromReferenceToPointCloud = identity;
	}

	return transformFromReferenceToPointCloud;
}


}

}

/* EOF */
