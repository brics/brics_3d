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

#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "Shape.h"

namespace BRICS_3D {

namespace RSG {

/**
 * Abstract interface for point cloud data in the scenegraph.
 */
class PointCloud : public Shape {
  public:
    PointCloud();

    virtual ~PointCloud();

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

