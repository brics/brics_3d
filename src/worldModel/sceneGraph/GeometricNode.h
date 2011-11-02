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

#ifndef GEOMETRICNODE_H
#define GEOMETRICNODE_H

#include "Node.h"
#include "TimeStamp.h"
#include "Shape.h"
#include "Attribute.h"

namespace BRICS_3D {

namespace RSG {

/**
 * @brief A leaf node in the robot scenegraph that carries any kind of 3D data.
 * 
 * The geometric node is a rather general container for <b>any</b> kind of 3D data. 
 * Possible data ranges from rather primitive shapes like boxes and cylinders to point clouds and meshes. 
 * 3D features like spin images etc. would be placed in to a geometric node too.
 */
class GeometricNode : public Node {

  public:

	typedef boost::shared_ptr<GeometricNode> GeometricNodePtr;
	typedef boost::shared_ptr<GeometricNode const> GeometricNodeConstPtr;

    GeometricNode();

    virtual ~GeometricNode();

    Shape::ShapePtr getShape() const
    {
        return shape;
    }

    TimeStamp getTimeStamp() const
    {
        return timeStamp;
    }

    void setShape(Shape::ShapePtr shape)
    {
        this->shape = shape;
    }

    void setTimeStamp(TimeStamp timeStamp)
    {
        this->timeStamp = timeStamp;
    }

    virtual void accept(INodeVisitor* visitor);

private:
    TimeStamp timeStamp;
    Shape::ShapePtr shape;

};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

