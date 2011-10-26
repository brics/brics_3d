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

#ifndef CYLINDER_H
#define CYLINDER_H

#include "Shape.h"
#include "core/Point3D.h"

namespace BRICS_3D {

namespace RSG {

/**
 * @brief Simple model of a 3D cylinder.
 *
 * The origin is considered in the center of each size value. The rotation axis is the Z axis.
 */
class Cylinder : public Shape {
public:

	typedef boost::shared_ptr<Cylinder> CylinderPtr;
	typedef boost::shared_ptr<Cylinder const> CylinderConstPtr;

	Cylinder();
	Cylinder(Coordinate radius, Coordinate height);

	virtual ~Cylinder();

    Coordinate getHeight() const
    {
        return height;
    }

    Coordinate getRadius() const
    {
        return radius;
    }

    void setHeight(Coordinate height)
    {
        this->height = height;
    }

    void setRadius(Coordinate radius)
    {
        this->radius = radius;
    }

private:
	Coordinate radius;
	Coordinate height;

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

