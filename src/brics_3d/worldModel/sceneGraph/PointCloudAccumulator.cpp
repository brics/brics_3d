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

#include "PointCloudAccumulator.h"
#include "PointCloud.h"
#include "brics_3d/core/Logger.h"
#include "brics_3d/core/HomogeneousMatrix44.h"

using brics_3d::Logger;

namespace brics_3d {

namespace rsg {

PointCloudAccumulator::PointCloudAccumulator(Node::NodePtr referenceNode) : INodeVisitor(downwards)  {
	accumulatedPointClouds = 0;
	this->referenceNode = referenceNode;
	reset();
}

PointCloudAccumulator::~PointCloudAccumulator() {

}

void PointCloudAccumulator::reset() {
	accumulatedPointClouds = new PointCloud3DIterator();
}

void PointCloudAccumulator::visit(Node* node){
	/* do nothing */
}

void PointCloudAccumulator::visit(Group* node){
	/* do nothing */
}

void PointCloudAccumulator::visit(Transform* node){
	/* do nothing */
}

void PointCloudAccumulator::visit(GeometricNode* node){
	assert (node != 0);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformFromReferenceToPointCloud = doGetTransformFromReferenceToPointCloud(node);

	/* check if geometry is a point cloud */
	Shape::ShapePtr resultShape;
	resultShape = node->getShape();
	rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pcResultContainer(new rsg::PointCloud<brics_3d::PointCloud3D>());
	pcResultContainer = boost::dynamic_pointer_cast<PointCloud<brics_3d::PointCloud3D> >(resultShape);
	if(pcResultContainer != 0) {
//		LOG(DEBUG) << "Adding point cloud to iterator with:";
//		LOG(DEBUG) << *pcResultContainer->data;
//		LOG(DEBUG) << *transformFromReferenceToPointCloud;
		accumulatedPointClouds->insert(pcResultContainer->data, transformFromReferenceToPointCloud);
	} else {
//		LOG(DEBUG) << " Geometric node, but no PointCloud3D";
	}
}

void PointCloudAccumulator::visit(Connection* connection) {
	/* do nothing */
}


IHomogeneousMatrix44::IHomogeneousMatrix44Ptr PointCloudAccumulator::doGetTransformFromReferenceToPointCloud(GeometricNode* node) {
	assert (node != 0);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformFromReferenceToPointCloud;

	/* we have to find the _shared_ pointer of the node - unless it is the root it will be one of the parents childs */
	if (node->getNumberOfParents() == 0) {
		LOG(WARNING) << "PointCloudAccumulator: Are you sure your root node is a GeometricNode (which is a leaf)?";

		/* so we take the identity */
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identity(new HomogeneousMatrix44());

		transformFromReferenceToPointCloud = identity;

	} else {
		unsigned int childIndex = 0;
		rsg::Node* parentNode;
		parentNode = node->getParent(0); //all parents will have the _same_ shared pointer so we pick the first parent
		Group* parentGroup =  dynamic_cast<Group*>(parentNode);
		assert (parentGroup != 0); // otherwise st.h went really wrong.
		childIndex = parentGroup->getChildIndex(node);
		Node::NodePtr nodeAsSharedPtr = parentGroup->getChild(childIndex);

		TimeStamp actualStamp;
		transformFromReferenceToPointCloud = getTransformBetweenNodes(nodeAsSharedPtr, referenceNode, node->getTimeStamp(), actualStamp); //For now we only query for creation time
	}

	return transformFromReferenceToPointCloud;
}


}

}

/* EOF */
