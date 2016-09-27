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

#ifndef RSG_CYLINDER_H
#define RSG_CYLINDER_H

#include "Shape.h"
#include "brics_3d/core/Point3D.h"

namespace brics_3d {

namespace rsg {

/**
 * @brief Simple model of a 3D cylinder.
 * @ingroup sceneGraph
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

	Shape::ShapeType getShapeType() {
		return Shape::Cylinder;
	}

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

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

