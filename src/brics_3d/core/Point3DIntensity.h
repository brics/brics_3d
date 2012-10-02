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

#ifndef BRICS_3D_POINT3DINTENSITY_H_
#define BRICS_3D_POINT3DINTENSITY_H_

#include "Point3DDecorator.h"

namespace brics_3d {

/**
 * Decoration layer for a Point3D that carries intensety information.
 */
class Point3DIntensity : public Point3DDecorator {
public:
	Point3DIntensity();
	Point3DIntensity(Point3D* point);
	Point3DIntensity(Point3D* point, double intensity);
	Point3DIntensity(Point3DIntensity* point);
	Point3DIntensity(const Point3DIntensity &point);
	Point3DIntensity& operator=(const Point3DIntensity &point);
	virtual ~Point3DIntensity();

	void setIntensity(double intensity);
	double getIntensity();

    virtual ColoredPoint3D* asColoredPoint3D(){
    	return point->asColoredPoint3D();
    }

    virtual Point3D* clone() const;

protected:
	double intensity;
};

}

#endif /* BRICS_3D_POINT3DINTENSITY_H_ */

/* EOF */
