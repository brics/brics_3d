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

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "core/IHomogeneousMatrix44.h"
#include "Group.h"
#include "TimeStamp.h"
#include <vector>
using std::vector;
using std::pair;

namespace BRICS_3D {

namespace RSG {

/**
 * @brief Determine the accumulated transform along a path of nodes.
 * @param nodePath A path of nodes from root descending.
 * @return Shared pointer to the accumulated transform.
 */
extern IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getGlobalTransformAlongPath(Node::NodePath nodePath);

/**
 * @brief Calculate the accumulated global transform for a node.
 *
 * In case the node is a transform node it will be taken into account too.
 * In case the node has multiple paths to root node, the first found path will be taken!
 * @param node The node to where the transform from root will calculated.
 * @return Shared pointer to the accumulated transform.
 */
extern IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getGlobalTransform(Node::NodePtr node);

/**
 * @brief A node that expresses a geometric transformation between its parents and children.
 */
class Transform : public Group {

  public:

	typedef boost::shared_ptr<RSG::Transform> TransformPtr;
	typedef boost::shared_ptr<RSG::Transform const> TransformConstPtr;

    Transform();

    virtual ~Transform();

    /**
     * @brief Add a new transform to the history.
     * @param newTransform The transform to be added.
     * @param timeStamp The times stamp that is associated with the transform. (So far this as no effect yet.)
     */
    void insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, TimeStamp timeStamp);

    /**
     * @brief Retrieve a transform that is closest to the specified timesStamp.
     * @param timeStamp No effect yet!
     * @return Shared pointer to the transform.
     */
    IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getTransform(TimeStamp timeStamp);

    /**
     * @brief Retrieve the latest/newest transform.
     * @return Shared pointer to the transform.
     */
    IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getLatestTransform();

    void accept(INodeVisitor* visitor);

  private:

    ///History of transforms. Each transform has an associated time stamp.
    vector< pair<IHomogeneousMatrix44::IHomogeneousMatrix44Ptr, TimeStamp> > history;

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

