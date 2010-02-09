/**
 * @file 
 * Point3DDecorator.cpp
 *
 * @date: Feb 9, 2010
 * @author: sblume
 */

#include "Point3DDecorator.h"

namespace BRICS_3D {

Point3DDecorator::Point3DDecorator(Point3D* point) :
			Point3D (point) { //TODO: check if this causes  inconsistency of coordinates
	this->decorate(point);
}

Point3DDecorator::~Point3DDecorator() {
	if (point != 0) {
//		delete point; //TODO: causes seg fault?
	}
}

Coordinate Point3DDecorator::getX() const {
	return point->getX(); //just a forward declaration
}

Coordinate Point3DDecorator::getY() const {
	return point->getY(); //just a forward declaration
}

Coordinate Point3DDecorator::getZ() const {
	return point->getZ(); //just a forward declaration
}

void Point3DDecorator::setX(Coordinate x) {
	point->setX(x); //just a forward declaration
}

void Point3DDecorator::setY(Coordinate y) {
	point->setY(y); //just a forward declaration
}

void Point3DDecorator::setZ(Coordinate z) {
	point->setZ(z); //just a forward declaration
}

void Point3DDecorator::homogeneousTransformation(IHomogeneousMatrix44 *transformation) {
	point->homogeneousTransformation(transformation); //just a forward declaration
}

void Point3DDecorator::decorate(Point3D* point) {
	this->point = point;
}

Point3D* Point3DDecorator::getPoint() {
	return point;
}


} // namespace BRICS_3D


/* EOF */
