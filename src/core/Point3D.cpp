/**
 * @file 
 * Point3D.cpp
 *
 * @date: Oct 14, 2009
 * @author: sblume
 */

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

namespace BRICS_3D {

Point3D::Point3D() {
	x = 0.0;
	y = 0.0;
	z = 0.0;
}

Point3D::Point3D(Point3D *point) {
	x = point->x;
	y = point->y;
	z = point->z;
}

Point3D::Point3D(double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

Point3D::~Point3D() {

}

void Point3D::getRawData(double *pointBuffer) {
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
				"Point3D operation exceeds the limit of datatype representation double.");
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
				"Point3D operation exceeds the limit of datatype representation double.");
	}

	return result;
}

Point3D Point3D::operator *(double scalar) {
	Point3D result;
	result.x = (this->x) * scalar;
	result.y = (this->y) * scalar;
	result.z = (this->z) * scalar;

	/* check if any value of x,y,z is infinity or not a number */
	if ((!(isfinite(result.x)) && (result.x != 0.0)) || (!(isfinite(result.y))
			&& (result.y != 0.0)) || (!(isfinite(result.z))
			&& (result.z != 0.0))) {
		throw runtime_error(
				"Point3D operation exceeds the limit of datatype representation double.");
	}

	return result;
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
	outStream << point.x << " " << point.y << " " << point.z;

	return outStream;
}

void Point3D::homogeneousTransformation(IHomogeneousMatrix44 *transformation) {

//	double* ptrHomogenousMatrix;
//	double homogenousMatrix[16];
	double *homogenousMatrix;
	double xTemp;
	double yTemp;
	double zTemp;

	transformation->getRawData(homogenousMatrix);
	//transformation->getRawData(ptrHomogenousMatrix);
	//memcpy(&homogenousMatrix, ptrHomogenousMatrix, 16*(sizeof(double)));
	//copy from pointer into array?!?

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
	x = xTemp + homogenousMatrix[12];
	y = yTemp + homogenousMatrix[13];
	z = zTemp + homogenousMatrix[14];
}

}

/* EOF */
