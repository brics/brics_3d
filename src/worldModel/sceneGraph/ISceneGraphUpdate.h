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

#ifndef ISCENEGRAPHUPDATE_H
#define ISCENEGRAPHUPDATE_H

#include "core/IHomogeneousMatrix44.h"
#include <vector>
using std::vector;
#include "TimeStamp.h"
#include "Shape.h"

namespace BRICS_3D { namespace RSG { class Attribute; }  } 

namespace BRICS_3D {

namespace RSG {

/**
 * @brief Abstract interface for all scenegraph related update functions.
 */
class ISceneGraphUpdate {
  public:
    /**
     * @brief Add a new node to the robot scenegraph.
     */
    void addNode(unsigned int parentId, unsigned int* assignedId, vector<Attribute> attributes);

    /**
     * @brief Add a group node to the robot scenegraph.
     * 
     * The group allows to create the graph structure as it has children, in contrast to a simple node.
     */
    void addGroup(unsigned int parentId, unsigned int* assignedId, vector<Attribute> attributes);

    /**
     * @brief Add a new transform node the the robot scenegraph.
     */
    void addTransformNode(unsigned int parentId, unsigned int* assignedId, vector<Attribute> attributes, BRICS_3D::IHomogeneousMatrix44* transform, TimeStamp timeStamp);

    /**
     * @brief Add a geometric node that contains 3D data to the robot scenegraph.
     * 
     * 
     */
    void addGeometricNode(unsigned int parentId, unsigned int* assignedId, Shape shape, vector<Attribute> attributes, TimeStamp timeStamp);

    /**
     * @brief Override the attributes of a node.
     */
    void setNodeAttributes(unsigned int id, vector<Attribute> newAttributes);

    /**
     * @brief Update the transformation of an existing TransformNode.
     */
    void setTransformation(unsigned int id, BRICS_3D::IHomogeneousMatrix44* transformation, TimeStamp timeStamp);

    /**
     * @brief Delete a node in the robot scenegraph.
     */
    void deleteNode(unsigned int id);

    /**
     * @brief Add a new parent-child relation between two nodes.
     */
    void addParent(unsigned int id, unsigned int parentId);

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

