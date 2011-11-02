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

#ifndef IWORLDMODELUPDATE_H
#define IWORLDMODELUPDATE_H

#include "SceneObject.h"

namespace BRICS_3D { namespace RSG { class GeometricNode; }  } 

namespace BRICS_3D {

/**
 * @brief Interface to get update elements in the 3D model.
 */
class IWorldModelUpdate {
  public:
    /**
     * @brief Adds a new SceneObject to the world model representation. Internally it will be added to the root node.
     */
    virtual void addSceneObject(SceneObject newObject, unsigned int* assignedId) = 0;

};

} // namespace BRICS_3D
#endif

/* EOF */

