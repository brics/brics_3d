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

#include "ColoredPoint3D.h"

#include <stdexcept>

using std::runtime_error;

namespace brics_3d {


ColoredPoint3D::ColoredPoint3D() :
					Point3DDecorator() {
	red = 0;
	green = 0;
	blue = 0;
}

ColoredPoint3D::ColoredPoint3D(Point3D* point) :
			Point3DDecorator (point) {
	red = 0;
	green = 0;
	blue = 0;
}

ColoredPoint3D::ColoredPoint3D(ColoredPoint3D* point) :
		Point3DDecorator (point) {
	red = point->red;
	green = point->green;
	blue = point->blue;
}

ColoredPoint3D::ColoredPoint3D(const ColoredPoint3D &point) :
		Point3DDecorator (point) {
	red = point.red;
	green = point.green;
	blue = point.blue;
}


ColoredPoint3D& ColoredPoint3D::operator=(const ColoredPoint3D &point) {
	Point3DDecorator::operator=(point);
	red = point.red;
	green = point.green;
	blue = point.blue;
}

ColoredPoint3D::ColoredPoint3D(Point3D* point, unsigned char red, unsigned char green, unsigned char blue) :
		Point3DDecorator (point) {
	this->red = red;
	this->green = green;
	this->blue = blue;
}

ColoredPoint3D::~ColoredPoint3D() {

}


istream& operator>>(istream &inStream, ColoredPoint3D &point) {
	unsigned int tmpColor;
	Coordinate tmpCoord;

	if (!inStream.good())
		throw runtime_error("ERROR: cannot read x coordinate of point.");
	inStream >> tmpCoord;
	point.setX(tmpCoord);
	if (!inStream.good())
		throw runtime_error("ERROR: cannot read y coordinate of point.");
	inStream >> tmpCoord;
	point.setY(tmpCoord);
	if (!inStream.good())
		throw runtime_error("ERROR: cannot read z coordinate of point.");
	inStream >> tmpCoord;
	point.setZ(tmpCoord);

	if (!inStream.good())
		throw runtime_error("ERROR: cannot read red color channel of point.");
	inStream >> tmpColor;
	point.red = static_cast<unsigned char>(tmpColor);

	if (!inStream.good())
		throw runtime_error("ERROR: cannot read green color channel of point.");
	inStream >> tmpColor;
	point.green = static_cast<unsigned char>(tmpColor);

	if (!inStream.good())
		throw runtime_error("ERROR: cannot read blue color channel of point.");
	inStream >> tmpColor;
	point.blue = static_cast<unsigned char>(tmpColor);

	return inStream;
}

ostream& operator<<(ostream &outStream, const ColoredPoint3D &point) {

	outStream << point.getX() << " " << point.getY() << " " << point.getZ();
	outStream << " " << static_cast<unsigned int>(point.red);
	outStream << " " << static_cast<unsigned int>(point.green);
	outStream << " " << static_cast<unsigned int>(point.blue);

	return outStream;
}

Point3D* ColoredPoint3D::clone() const {
	return new ColoredPoint3D(point->clone(), red, green, blue);
}

ColoredPoint3D* ColoredPoint3D::asColoredPoint3D() {
	return this;
}

}

/* EOF */
