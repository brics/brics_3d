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

#include "Point3DNormal.h"

namespace brics_3d {


Point3DNormal::Point3DNormal() :
		Point3DDecorator() {
	this->normal = Normal3D(0.0, 0.0, 0.0);
}

Point3DNormal::Point3DNormal(Point3D* point) :
		Point3DDecorator(point) {
	this->normal = Normal3D(0.0, 0.0, 0.0);
}

Point3DNormal::Point3DNormal(Point3D* point, Normal3D normal) :
		Point3DDecorator(point) {
	this->normal = normal;
}

Point3DNormal::Point3DNormal(Point3DNormal* point) :
		Point3DDecorator(point) {
	this->normal = point->getNormal();
}

Point3DNormal::Point3DNormal(const Point3DNormal &point):
		Point3DDecorator(point) {
	this->normal = point.normal;
}

Point3DNormal& Point3DNormal::operator=(const Point3DNormal &point) {
	Point3DDecorator::operator=(point);
	normal = point.normal;
}

Point3DNormal::~Point3DNormal() {

}

void Point3DNormal::homogeneousTransformation(IHomogeneousMatrix44 *transformation) {
	 Point3DDecorator::homogeneousTransformation(transformation);
	 this->normal.homogeneousTransformation(transformation);
}

void Point3DNormal::setNormal(Normal3D normal) {
	this->normal = normal;
}

Normal3D Point3DNormal::getNormal() {
	return this->normal;
}

Point3D* Point3DNormal::clone() const {
	return new Point3DNormal(point->clone(), normal);
}

}  // namespace brics_3d


/* EOF */
