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

namespace brics_3d { namespace rsg { class Attribute; }  } 

namespace brics_3d {

namespace rsg {

/**
 * @brief Abstract interface for all scene graph related update functions.
 * @ingroup sceneGraph
 */
class ISceneGraphUpdate {
  public:

	/**
	 * @brief Add a new Node to the robot scene graph.
	 *
	 * @param[in] parentId ID of the parent node. To add more parents use the brics_3d::rsg::ISceneGraphUpdate::addParent() function.
	 * @param[in,out] assignedId The ID that was assigned internally.
	 *                           In case the forcedId flag has been set to true it will used instead of the internal ID assignment.
	 *                           Use this functio with care. There are are some internal checks if the manually assigned is valid
	 *                           but do not rely on this in a distibuted setting.
	 * @param[in] attributes A set of attributs that will be set for the node.
	 * @param[in] forcedId If set to true the ID defined in assignedId will be taken instead of an intanally generated one. This is
	 *                     in particular useful if distibuted scene graphs propagate updates and ensures IDs are not re-created.
	 * @return True if a new Node was sucessfully added to the scene graph.
	 */
	virtual bool addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, bool forcedId = false) = 0;

    /**
     * @brief Add a Group node to the robot scenegraph.
     * 
     * The group allows to create the graph structure as it has children, in contrast to a simple node.
	 *
	 * @param[in] parentId ID of the parent node. To add more more parents use the brics_3d::rsg::ISceneGraphUpdate::addParent() function.
	 * @param[in,out] assignedId The ID that was assigned internally.
	 *                           In case the forcedId flag has been set to true it will used instead of the internal ID assignment.
	 *                           Use this functio with care. There are are some internal checks if the manually assigned is valid
	 *                           but do not rely on this in a distibuted setting.
	 * @param[in] attributes A set of attributs that will be set for the node.
	 * @param[in] forcedId If set to true the ID defined in assignedId will be taken instead of an intanally generated one. This is
	 *                     in particular useful if distibuted scene graphs propagate updates and ensures IDs are not re-created.
	 * @return True if a new Group was sucessfully added to the scene graph.
	 */
	virtual bool addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, bool forcedId = false) = 0;

    /**
     * @brief Add a new Transform node the the robot scene graph.
	 *
	 * Internally brics_3d::rsg::Transform is used that is able to maintain a temporal cache of the trnasform data.
	 *
	 * @param[in] parentId ID of the parent node. To add more more parents use the brics_3d::rsg::ISceneGraphUpdate::addParent() function.
	 * @param[in,out] assignedId The ID that was assigned internally.
	 *                           In case the forcedId flag has been set to true it will used instead of the internal ID assignment.
	 *                           Use this functio with care. There are are some internal checks if the manually assigned is valid
	 *                           but do not rely on this in a distibuted setting.
	 * @param[in] attributes A set of attributs that will be set for the node.
	 * @param transform The actual transform data that will be (initially) inserted. You can later insert more with brics_3d::rsg::ISceneGraphUpdate::setTransform() .
	 * @param timeStamp Time stamp associated with the transform.
	 * @param forcedId If set to true the ID defined in assignedId will be taken instead of an intanally generated one. This is
	 *                     in particular useful if distibuted scene graphs propagate updates and ensures IDs are not re-created.
	 * @return True if a new Transform was sucessfully added to the scene graph.
	 */
	virtual bool addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId = false) = 0;

    /**
     * @brief Add a GeometricNode that contains 3D data to the robot scene graph.
	 *
	 * @param[in] parentId ID of the parent node. To add more more parents use the brics_3d::rsg::ISceneGraphUpdate::addParent() function.
	 * @param[in,out] assignedId The ID that was assigned internally.
	 *                           In case the forcedId flag has been set to true it will used instead of the internal ID assignment.
	 *                           Use this functio with care. There are are some internal checks if the manually assigned is valid
	 *                           but do not rely on this in a distibuted setting.
	 * @param[in] attributes A set of attributs that will be set for the node.
	 * @param shape The geometic Shape to be contained the the GeometricNode. Multiple shape types are possible like a Box, a Cylinder a Mesh or a PointCloud for instance.
	 * @param timeStamp Time stamp assiciated with the geometic Shape.
	 * @param forcedId If set to true the ID defined in assignedId will be taken instead of an intanally generated one. This is
	 *                     in particular useful if distibuted scene graphs propagate updates and ensures IDs are not re-created.
	 * @return True if a new GeometricNode was sucessfully added to the scen egraph.
	 */
	virtual bool addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId = false) = 0;

    /**
     * @brief Override the attributes of a node.
	 *
	 * @param id ID the defines the node to be updated.
	 * @param newAttributes The new attributes. THe old one will be overidden by tis
	 * @return True on success.
	 */
	virtual bool setNodeAttributes(unsigned int id, vector<Attribute> newAttributes) = 0;

    /**
     * @brief Update the transformation of an existing Transform node.
	 *
	 * @param id ID the defines the Transform node to be updated.
	 * @param transform The transform data that will be inserted in to the cacje of the Transform.
	 * @param timeStamp Time stamp associated with the transform.
	 * @return True on success.
	 */
	virtual bool setTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) = 0;

    /**
     * @brief Delete a node in the robot scenegraph.
     *
	 * @param id ID the defines the node to be deleted.
	 * @return True on success.
	 */
	virtual bool deleteNode(unsigned int id) = 0;

    /**
     * @brief Add a new parent-child relation between two nodes.
	 *
	 * @param id ID of the node
	 * @param parentId ID of the parent node. Must not be a <b>leave</b> like a Node or a GeometricNode
	 * @return True on success.
	 */
	virtual bool addParent(unsigned int id, unsigned int parentId) = 0;

    //void delete parent

};

} // namespace brics_3d::RSG

} // namespace brics_3d
#endif

/* EOF */

