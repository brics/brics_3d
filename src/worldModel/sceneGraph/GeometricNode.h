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

#ifndef GEOMETRICNODE_H
#define GEOMETRICNODE_H

#include "Node.h"
#include "TimeStamp.h"
#include "Shape.h"
#include "Attribute.h"

namespace BRICS_3D {

namespace RSG {

/**
 *  @brief A leaf node in the robot scenegraph that carries any kind of 3D data.
 * 
 * The geometric node is a rather general container for <b>any</b> kind of 3D data. 
 * Possible data ranges from rather primitive shapes like boxes and cylinders to point clouds and meshes. 
 * 3D features like spin images etc. would be placed in to a geometric node too.
 */
class GeometricNode : public Node, public Attribute {
  public:
    TimeStamp timeStamp;


  private:
    Shape shape;


  public:
    GeometricNode();

    virtual ~GeometricNode();

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

