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
#include "core/Logger.h"
#include "core/HomogeneousMatrix44.h"

using BRICS_3D::Logger;

namespace BRICS_3D {

namespace RSG {

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
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformReferenceToPointCloud;

	/* we have to find the _shared_ pointer of node - unless it is the root it will be one or the parents childs */
	if (node->getNumberOfParents() == 0) {
		LOG(WARNING) << "PointCloudAccumulator: Are you sure your root node is a GeometricNode (which is a leaf)?";

		/* so we take the identity */
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identity(new HomogeneousMatrix44());

		transformReferenceToPointCloud = identity;

	} else {
		unsigned int childIndex = 0;
		RSG::Node* parentNode;
		parentNode = node->getParent(0); //all parents will have the _same_ shared pointer so we pick the first parent
		Group* parentGroup =  dynamic_cast<Group*>(parentNode);
		assert (parentGroup != 0); // otherwise st.h went really wrong.
		childIndex = parentGroup->getChildIndex(node);
		Node::NodePtr nodeAsSharedPtr = parentGroup->getChild(childIndex);

		transformReferenceToPointCloud = getTransformBetweenNodes(nodeAsSharedPtr, referenceNode);
	}

	/* check if geometry is a point cloud */
	Shape::ShapePtr resultShape;
	resultShape = node->getShape();
	RSG::PointCloud<BRICS_3D::PointCloud3D>::PointCloudPtr pcResultContainer(new RSG::PointCloud<BRICS_3D::PointCloud3D>());
	pcResultContainer = boost::dynamic_pointer_cast<PointCloud<BRICS_3D::PointCloud3D> >(resultShape);
	if(pcResultContainer != 0) {
		LOG(DEBUG) << "Adding point cloud to iterator with:";
		LOG(DEBUG) << *pcResultContainer->data;
		LOG(DEBUG) << *transformReferenceToPointCloud;
		accumulatedPointClouds->insert(pcResultContainer->data, transformReferenceToPointCloud);
	} else {
		LOG(DEBUG) << " Geometric node, but no PointCloud3D";
	}
}


}

}

/* EOF */
