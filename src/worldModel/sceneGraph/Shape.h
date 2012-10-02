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

#ifndef RSG_SHAPE_H
#define RSG_SHAPE_H

#include <boost/shared_ptr.hpp>

namespace brics_3d { namespace rsg { class GeometricNode; }  } 

namespace brics_3d {

namespace rsg {

/**
 * @brief A generic shape that can by carried by a GeometricNode.
 * @ingroup sceneGraph
 *
 * For basic shapes like boxes etc. the origin is assumed to be in the center of the shape.
 */
class Shape {

public:
	typedef boost::shared_ptr<Shape> ShapePtr;
	typedef boost::shared_ptr<Shape const> ShapeConstPtr;

	Shape();

	virtual ~Shape();

};

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

