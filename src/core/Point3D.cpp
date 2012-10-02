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

#include "Point3D.h"

#include <math.h>
#include <cmath>
#include <stdexcept>

using std::runtime_error;

#ifdef WIN32
#define isfinite(x) (1)
#else
using std::isfinite;
#endif

namespace brics_3d {

Point3D::Point3D() {
	this->x = 0.0;
	this->y = 0.0;
	this->z = 0.0;
}

Point3D::Point3D(Point3D *point) {
	this->x = point->getX();
	this->y = point->getY();
	this->z = point->getZ();
}

Point3D::Point3D(const Point3D &point) {
	this->x = point.getX();
	this->y = point.getY();
	this->z = point.getZ();
}


Point3D::Point3D(Coordinate x, Coordinate y, Coordinate z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

Point3D::~Point3D() {

}

Coordinate Point3D::getX() const {
	return this->x;
}

Coordinate Point3D::getY() const {
	return this->y;
}

Coordinate Point3D::getZ() const {
	return this->z;
}

void Point3D::setX(Coordinate x) {
	this->x = x;
}
void Point3D::setY(Coordinate y) {
	this->y = y;
}
void Point3D::setZ(Coordinate z) {
	this->z = z;
}

void Point3D::getRawData(Coordinate *pointBuffer) {
	pointBuffer[0] = x;
	pointBuffer[1] = y;
	pointBuffer[2] = z;
}

Point3D Point3D::operator +(const Point3D *point) {
	Point3D result;
	result.x = this->x + point->x;
	result.y = this->y + point->y;
	result.z = this->z + point->z;

	/* check if any value of x,y,z is infinity or not a number */
	if ((!(isfinite(result.x)) && (result.x != 0.0)) || (!(isfinite(result.y))
			&& (result.y != 0.0)) || (!(isfinite(result.z))
			&& (result.z != 0.0))) {
		throw runtime_error(
				"Point3D operation exceeds the limit of Coordinate datatype representation.");
	}

	return result;
}

Point3D Point3D::operator -(const Point3D *point) {
	Point3D result;
	result.x = this->x - point->x;
	result.y = this->y - point->y;
	result.z = this->z - point->z;

	/* check if any value of x,y,z is infinity or not a number */
	if ((!(isfinite(result.x)) && (result.x != 0.0)) || (!(isfinite(result.y))
			&& (result.y != 0.0)) || (!(isfinite(result.z))
			&& (result.z != 0.0))) {
		throw runtime_error(
				"Point3D operation exceeds the limit of Coordinate datatype representation.");
	}

	return result;
}

Point3D Point3D::operator *(double scalar) {
	Point3D result;
	result.x = (this->x) * static_cast<Coordinate>(scalar);
	result.y = (this->y) * static_cast<Coordinate>(scalar);
	result.z = (this->z) * static_cast<Coordinate>(scalar);

	/* check if any value of x,y,z is infinity or not a number */
	if ((!(isfinite(result.x)) && (result.x != 0.0)) || (!(isfinite(result.y))
			&& (result.y != 0.0)) || (!(isfinite(result.z))
			&& (result.z != 0.0))) {
		throw runtime_error(
				"Point3D operation exceeds the limit of Coordinate datatype representation.");
	}

	return result;
}

Point3D& Point3D::operator=(const Point3D &point) {
	if (this == &point) { //case of self assignment
		return *this;
	}
	this->x = point.getX();
	this->y = point.getY();
	this->z = point.getZ();
	return *this;
}

void Point3D::homogeneousTransformation(IHomogeneousMatrix44 *transformation) { //TODO throw exception if values exceed limits
	// NOTE: currently everything is done in "double" and afterward transformed to "Coordinate"
	// This might involve implicit conversions between types (float to double e.g.) for each transformation...
	const double *homogenousMatrix;
	double xTemp;
	double yTemp;
	double zTemp;

	homogenousMatrix = transformation->getRawData();

	/*
	 * layout:
	 * 0 4 8  12
	 * 1 5 9  13
	 * 2 6 10 14
	 * 3 7 11 15
	 */

	/* rotate */;
	xTemp = x * homogenousMatrix[0] + y * homogenousMatrix[4] + z * homogenousMatrix[8];
	yTemp = x * homogenousMatrix[1] + y * homogenousMatrix[5] + z * homogenousMatrix[9];
	zTemp = x * homogenousMatrix[2] + y * homogenousMatrix[6] + z * homogenousMatrix[10];

	/* translate */
	x = static_cast<Coordinate>(xTemp + homogenousMatrix[12]);
	y = static_cast<Coordinate>(yTemp + homogenousMatrix[13]);
	z = static_cast<Coordinate>(zTemp + homogenousMatrix[14]);
}

istream& operator>>(istream &inStream, Point3D &point) {
	if (!inStream.good())
		throw runtime_error("ERROR: cannot read x coordinate of point.");
	inStream >> point.x;
	if (!inStream.good())
		throw runtime_error("ERROR: cannot read y coordinate of point.");
	inStream >> point.y;
	if (!inStream.good())
		throw runtime_error("ERROR: cannot read z coordinate of point.");
	inStream >> point.z;

	return inStream;
}

ostream& operator<<(ostream &outStream, const Point3D &point) {
	outStream << point.getX() << " " << point.getY() << " " << point.getZ();

	return outStream;
}

Point3D* Point3D::clone() const {
	return new Point3D(*this);
}

ColoredPoint3D* Point3D::asColoredPoint3D() {
	return 0;
}

}

/* EOF */
