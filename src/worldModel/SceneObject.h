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

