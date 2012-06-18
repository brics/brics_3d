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

#ifndef BOX_H
#define BOX_H

#include "Shape.h"
#include "core/Point3D.h"

namespace BRICS_3D {

namespace RSG {

/**
 * @brief Simple model of a 3D box.
 * @ingroup sceneGraph
 *
 * The origin is considered in the center of each size value.
 */
class Box : public Shape {
public:

	typedef boost::shared_ptr<Box> BoxPtr;
	typedef boost::shared_ptr<Box const> BoxConstPtr;

    Box();
    Box(Coordinate sizeX, Coordinate sizeY, Coordinate sizeZ);

    virtual ~Box();

    Coordinate getSizeX() const
    {
        return sizeX;
    }

    Coordinate getSizeY() const
    {
        return sizeY;
    }

    Coordinate getSizeZ() const
    {
        return sizeZ;
    }

    void setSizeX(Coordinate sizeX)
    {
        this->sizeX = sizeX;
    }

    void setSizeY(Coordinate sizeY)
    {
        this->sizeY = sizeY;
    }

    void setSizeZ(Coordinate sizeZ)
    {
        this->sizeZ = sizeZ;
    }

private:
    Coordinate sizeX;
    Coordinate sizeY;
    Coordinate sizeZ;

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

