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

#ifndef POINTCLOUDACCUMULATOR_H_
#define POINTCLOUDACCUMULATOR_H_

#include "Node.h"
#include "Group.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "INodeVisitor.h"

#include "core/PointCloud3DIterator.h"

namespace brics_3d {

namespace rsg {

/**
 * @brief Scenegraph visitor that collects all point clouds in a sub-graph and returns a point iterator brics_3d::IPoint3DIterator.
 *
 * This visitor will add every found point cloud visited while traversing a sub-graph (where accept() is called).
 * Each point cloud will be interpreted ralative to the frame that is valid for the scenegraph node specified by referenceNode.
 * The resulting iterator will handle this interpretation intrinsically as long you use the getters like brics_3d::IPoint3DIterator::getX().
 *
 * @ingroup sceneGraph
 */
class PointCloudAccumulator : public INodeVisitor {
public:

	/**
	 * @brief Constructor with reference frame defined by the node.
	 * @param referenceNode The Cartesian frame that is valid for the node will be used as reference to interpret the 3D points.
	 */
	PointCloudAccumulator(Node::NodePtr referenceNode);

	/**
	 * @brief Default destructor.
	 */
	virtual ~PointCloudAccumulator();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);

	/**
	 * @brief Reinitialize the internal iterator. Invoke this before travasing more then once. Otherwise data will be appended.
	 */
	virtual void reset();

	/**
	 * @brief Get the iterator that contains all point cloud in the subraph after a traversal.
	 *
	 * A traversal is triggered by calling brics_3d::rsg::Node::accept().
	 *
	 * @return The iterator handle.
	 */
	IPoint3DIterator* getAccumulatedPointClouds () {
		return accumulatedPointClouds;
	}

	/**
	 * @brief Overridable "template method" (for example for an ID aware version) for for resolving the transfrom between the reference frame and the fame valid in the currently visited GeometricNode.
	 * @param node The currently visited GeometricNode.
	 */
	virtual IHomogeneousMatrix44::IHomogeneousMatrix44Ptr doGetTransformFromReferenceToPointCloud(GeometricNode* node);

protected:

	PointCloud3DIterator* accumulatedPointClouds;
	Node::NodePtr referenceNode;

};

}

}

#endif /* POINTCLOUDACCUMULATOR_H_ */

/* EOF */
