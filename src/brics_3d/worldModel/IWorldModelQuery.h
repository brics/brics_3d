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

#ifndef BRICS_3D_IWORLDMODELQUERY_H
#define BRICS_3D_IWORLDMODELQUERY_H

#include "brics_3d/core/IHomogeneousMatrix44.h"
#include <vector>
#include "brics_3d/worldModel/sceneGraph/Id.h"
using std::vector;

namespace brics_3d { namespace rsg { class Attribute; }  } 
namespace brics_3d { class SceneObject; } 
namespace brics_3d { namespace rsg { class GeometricNode; }  } 

namespace brics_3d {

/**
 * @brief "High level" interface to get access to elements in the 3D model.
 * @ingroup sceneGraph
 */
class IWorldModelQuery {
  public:
    virtual void getSceneObjects(vector<rsg::Attribute> attributes, vector<SceneObject>& results) = 0;

    virtual void getCurrentTransform(rsg::Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform)= 0;

};

} // namespace brics_3d
#endif

/* EOF */

