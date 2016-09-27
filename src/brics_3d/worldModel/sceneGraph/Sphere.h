/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2014, KU Leuven
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

#ifndef RSG_SPHERE_H_
#define RSG_SPHERE_H_

#include "Shape.h"
#include "brics_3d/core/Point3D.h"
#include "brics_3d/core/Units.h"

namespace brics_3d {
namespace rsg {

/**
 * @brief Simple model of a 3D sphere.
 * @ingroup sceneGraph
 *
 * The origin is considered in the center of each size value.
 */
class Sphere : public Shape {
public:

	typedef boost::shared_ptr<Sphere> SpherePtr;
	typedef boost::shared_ptr<Sphere const> SphereConstPtr;

	Sphere();
	Sphere(Coordinate radius, Units::DistanceUnit unit = Units::Meter);
	virtual ~Sphere();

	Shape::ShapeType getShapeType() {
		return Shape::Sphere;
	}

    Coordinate getRadius() const
    {
        return radius;
    }

    void setRadius(Coordinate radius,  Units::DistanceUnit unit = Units::Meter)
    {
         this->radius = Units::distanceToMeters(radius, unit);
    }

private:

    /// The radius of the sphere in [m].
	Coordinate radius;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_SPHERE_H_ */

/* EOF */
