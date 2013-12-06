/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2013, KU Leuven
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

#ifndef BRICS_3D_POINT3DNORMAL_H_
#define BRICS_3D_POINT3DNORMAL_H_

#include "Point3DDecorator.h"
#include "Normal3D.h"

namespace brics_3d {

/**
 * Decoration layer for a Point3D that carries normal vector information.
 */
class Point3DNormal : public Point3DDecorator  {

	Point3DNormal();
	Point3DNormal(Point3D* point);
	Point3DNormal(Point3D* point, Normal3D normal);
	Point3DNormal(Point3DNormal* point);
	Point3DNormal(const Point3DNormal &point);
	Point3DNormal& operator=(const Point3DNormal &point);
	virtual ~Point3DNormal();

	void homogeneousTransformation(IHomogeneousMatrix44 *transformation);

	void setNormal(Normal3D normal);
	Normal3D getNormal();

    virtual ColoredPoint3D* asColoredPoint3D(){
    	return point->asColoredPoint3D();
    }

    virtual Point3D* clone() const;

protected:
	Normal3D normal;

};

}

#endif /* BRICS_3D_POINT3DNORMAL_H_ */

/* EOF */
