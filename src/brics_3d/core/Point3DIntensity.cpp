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

#include "Point3DIntensity.h"

namespace brics_3d {

Point3DIntensity::Point3DIntensity() :
		Point3DDecorator() {
	this->intensity = 0.0;
}

Point3DIntensity::Point3DIntensity(Point3D* point) :
		Point3DDecorator(point) {
	this->intensity = 0.0;
}

Point3DIntensity::Point3DIntensity(Point3D* point, double intensity) :
		Point3DDecorator(point) {
	this->intensity = intensity;
}

Point3DIntensity::Point3DIntensity(Point3DIntensity* point) :
		Point3DDecorator(point) {
	this->intensity = point->getIntensity();
}

Point3DIntensity::Point3DIntensity(const Point3DIntensity &point):
		Point3DDecorator(point) {
	this->intensity = point.intensity;
}

Point3DIntensity& Point3DIntensity::operator=(const Point3DIntensity &point) {
	Point3DDecorator::operator=(point);
	this->intensity = point.intensity;
}

Point3DIntensity::~Point3DIntensity() {

}

void Point3DIntensity::setIntensity(double intensity) {
	this->intensity = intensity;
}

double Point3DIntensity::getIntensity() {
	return this->intensity;
}

Point3D* Point3DIntensity::clone() const {
	return new Point3DIntensity(point->clone(), intensity);
}

}

/* EOF */
