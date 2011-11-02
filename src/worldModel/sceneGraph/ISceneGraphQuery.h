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

#ifndef ISCENEGRAPHQUERY_H
#define ISCENEGRAPHQUERY_H

#include "core/IHomogeneousMatrix44.h"
#include <vector>
using std::vector;
#include "TimeStamp.h"

namespace BRICS_3D { namespace RSG { class Attribute; }  } 
namespace BRICS_3D { namespace RSG { class Shape; }  } 

namespace BRICS_3D {

namespace RSG {

/**
 * @brief Abstract interface for all scenegraph related query functions.
 */
class ISceneGraphQuery {
  public:
    /**
     * @brief Find all nodes that have at least the specified attributes.
     */
    void getNodes(vector<Attribute> attributes, vector<unsigned int>* ids);

    /**
     * @brief Get the attributes of a node.
     */
    void getNodeAttributes(unsigned int id, vector<Attribute>* attributes);

    /**
     * @brief Get all the parent IDs of a certain node.
     */
    virtual bool getNodeParents(unsigned int id, vector<unsigned int>& parentIds) = 0;

//    void getGroupChildren

    /**
     * @brief Get the transform of a TransformNode at a certain time.
     */
    void getTransform(unsigned int id, TimeStamp timeStamp, BRICS_3D::IHomogeneousMatrix44* transform);

    /**
     * @brief Get the data of a GeometryNode.
     */
    void getGeometry(unsigned int id, Shape* shape, TimeStamp* timeStamp);

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

