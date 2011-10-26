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

