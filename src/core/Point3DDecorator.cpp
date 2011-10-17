/**
 * @file 
 * Point3DDecorator.cpp
 *
 * @date: Feb 9, 2010
 * @author: sblume
 */

#include "Point3DDecorator.h"

namespace BRICS_3D {

Point3DDecorator::Point3DDecorator() {
	this->point = new Point3D(); //create default point (0,0,0)
}

Point3DDecorator::Point3DDecorator(Point3D* point) :
			Point3D (point) {
//	this->decorate(point);
	this->point = new Point3D(point);
}

Point3DDecorator::Point3DDecorator(Point3DDecorator* point) {
	this->point = new Point3D(point);
//	this->decorate(new Point3D(point));
}

Point3DDecorator::Point3DDecorator(const Point3DDecorator &point) {
	this->point = new Point3D(point);
}

Point3DDecorator& Point3DDecorator::operator=(Point3DDecorator &point) {
	*(this->point) = point;
}

Point3DDecorator::~Point3DDecorator() {
	if (point != 0) {
		delete point;
		point = 0;
	}
}

Point3D Point3DDecorator::operator+(const Point3D *point) {
	return this->point->operator +(point); //just a forward declaration
}

Point3D Point3DDecorator::operator-(const Point3D *point) {
	return this->point->operator -(point); //just a forward declaration
}

Point3D Point3DDecorator::operator *(double scalar) {
	return this->point->operator *(scalar); //just a forward declaration
}

Point3D& Point3DDecorator::operator=(const Point3D &point) {
	/*
	 * This is a forward declaration, that prevents from assigning results to the super class of the decorator,
	 * rather than to the next skin/layer/wrapper of the decorator. Otherwise data inconsistency might occur!
	 */
	return this->point->operator =(point);
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

void Point3DDecorator::getRawData(Coordinate *pointBuffer) {
	point->getRawData(pointBuffer); //just a forward declaration
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
