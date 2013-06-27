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

#ifndef RSG_ISCENEGRAPHQUERY_H
#define RSG_ISCENEGRAPHQUERY_H

#include "brics_3d/core/IHomogeneousMatrix44.h"
#include "brics_3d/core/ITransformUncertainty.h"
#include <vector>
using std::vector;
#include "TimeStamp.h"
#include "Shape.h"

namespace brics_3d { namespace rsg { class Attribute; }  } 

namespace brics_3d {

namespace rsg {

/**
 * @brief Abstract interface for all scenegraph related query functions.
 * @ingroup sceneGraph
 */
class ISceneGraphQuery {
  public:
    /**
     * @brief Find all nodes that have at least the specified attributes.
     */
	virtual bool getNodes(vector<Attribute> attributes, vector<unsigned int>& ids) = 0;

    /**
     * @brief Get the attributes of a node.
     */
	virtual bool getNodeAttributes(unsigned int id, vector<Attribute>& attributes) = 0;

    /**
     * @brief Get all the parent IDs of a certain node.
     */
    virtual bool getNodeParents(unsigned int id, vector<unsigned int>& parentIds) = 0;

    /**
     * @brief Gett all cild IDs for a certain group node.
     */
    virtual bool getGroupChildren(unsigned int id, vector<unsigned int>& childIds) = 0;

    /**
     * @brief Get the transform of a TransformNode at a certain time.
     */
    virtual bool getTransform(unsigned int id, TimeStamp timeStamp, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform) = 0;

    /**
     * @brief Get the transform and uncertainty data of an UncertainTransformNode at a certain time.
     */
    virtual bool getUncertainTransform(unsigned int id, TimeStamp timeStamp, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform, ITransformUncertainty::ITransformUncertaintyPtr &uncertainty) = 0;


    /**
     * @brief Get the data of a GeometryNode.
     */
    virtual bool getGeometry(unsigned int id, Shape::ShapePtr& shape, TimeStamp& timeStamp) = 0;

};

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

