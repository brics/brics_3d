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
	virtual bool addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes) = 0;

    /**
     * @brief Add a group node to the robot scenegraph.
     * 
     * The group allows to create the graph structure as it has children, in contrast to a simple node.
     */
	virtual bool addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes) = 0;

    /**
     * @brief Add a new transform node the the robot scenegraph.
     */
	virtual bool addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) = 0;

    /**
     * @brief Add a geometric node that contains 3D data to the robot scenegraph.
     */
	virtual bool addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp) = 0;

    /**
     * @brief Override the attributes of a node.
     */
	virtual bool setNodeAttributes(unsigned int id, vector<Attribute> newAttributes) = 0;

    /**
     * @brief Update the transformation of an existing TransformNode.
     */
	virtual bool setTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) = 0;

    /**
     * @brief Delete a node in the robot scenegraph.
     */
	bool deleteNode(unsigned int id); //  = 0 FIXME

    /**
     * @brief Add a new parent-child relation between two nodes.
     */
	virtual bool addParent(unsigned int id, unsigned int parentId) = 0;

    //void delete parent

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

