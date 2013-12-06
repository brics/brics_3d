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

#include "Normal3D.h"

namespace brics_3d {


Normal3D::Normal3D() {

}

Normal3D::~Normal3D() {

}

Normal3D::Normal3D(Coordinate x, Coordinate y, Coordinate z) : Vector3D(x,y,z) {

}

Normal3D::Normal3D(Normal3D* normal) {
	this->x = normal->getX();
	this->y = normal->getY();
	this->z = normal->getZ();
}

void Normal3D::homogeneousTransformation(IHomogeneousMatrix44 *transformation) {
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
	x = static_cast<Coordinate>(xTemp);
	y = static_cast<Coordinate>(yTemp);
	z = static_cast<Coordinate>(zTemp);

	/* translate: nothing to do here */

}

Normal3D& Normal3D::operator=(const Normal3D &normal) {
	if (this == &normal) { //case of self assignment
		return *this;
	}
	this->x = normal.getX();
	this->y = normal.getY();
	this->z = normal.getZ();
	return *this;
}

}

/* EOF */
