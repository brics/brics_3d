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

    //void delete parent

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

