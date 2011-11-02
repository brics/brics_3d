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

#ifndef SCENEOBJECT_H
#define SCENEOBJECT_H

#include "core/IHomogeneousMatrix44.h"
#include "sceneGraph/Shape.h"
#include <vector>
using std::vector;

namespace BRICS_3D { namespace RSG { class GeometricNode; }  } 
namespace BRICS_3D { namespace RSG { class Attribute; }  } 

namespace BRICS_3D {

class SceneObject {

  public:

    SceneObject();

    virtual ~SceneObject();

    SceneObject(SceneObject & source);

    SceneObject(const SceneObject & source);

    SceneObject & operator=(SceneObject & source);

    SceneObject & operator=(const SceneObject & source);

    friend  ostream & operator<<(ostream & os, const SceneObject & x);


    unsigned int id;

    unsigned int parentId;

    IHomogeneousMatrix44* transform;

    BRICS_3D::RSG::Shape* shape;

    vector<RSG::Attribute> attributes;

};

} // namespace BRICS_3D
#endif

/* EOF */

