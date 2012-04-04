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

#include "BoxROIExtractor.h"
#include "core/HomogeneousMatrix44.h"

namespace BRICS_3D {

BoxROIExtractor::BoxROIExtractor() {
	this->sizeX = 1.0;
	this->sizeY = 1.0;
	this->sizeZ = 1.0;
}

BoxROIExtractor::BoxROIExtractor(Coordinate sizeX, Coordinate sizeY, Coordinate sizeZ) {
	this->sizeX = sizeX;
	this->sizeY = sizeY;
	this->sizeZ = sizeZ;
}

BoxROIExtractor::~BoxROIExtractor() {

}

void BoxROIExtractor::filter(PointCloud3D* originalPointCloud, PointCloud3D* resultPointCloud) {
	if (boxOrigin == 0 || boxOrigin->isIdentity()) { //lazy evaluation...
		for (unsigned int i = 0; i < originalPointCloud->getSize(); ++i) {
			Point3D tmpPoint = (*originalPointCloud->getPointCloud())[i]; //TODO: prevent slicing
			if (tmpPoint.getX() >= -sizeX/2 && tmpPoint.getX() <= sizeX/2 &&
					tmpPoint.getY() >= -sizeY/2 && tmpPoint.getY() <= sizeY/2 &&
					tmpPoint.getZ() >= -sizeZ/2 && tmpPoint.getZ() <= sizeZ/2) {
				resultPointCloud->addPoint(tmpPoint);
			}
		}
	} else {
		HomogeneousMatrix44* inverseOrigin = new HomogeneousMatrix44();
		*inverseOrigin = *(boxOrigin.get());
		inverseOrigin->inverse();
		for (unsigned int i = 0; i < originalPointCloud->getSize(); ++i) {
			Point3D tmpPoint = (*originalPointCloud->getPointCloud())[i]; //TODO: prevent slicing
			tmpPoint.homogeneousTransformation(inverseOrigin); // move all points to the origin and then compare
			if (tmpPoint.getX() >= -sizeX/2 && tmpPoint.getX() <= sizeX/2 &&
					tmpPoint.getY() >= -sizeY/2 && tmpPoint.getY() <= sizeY/2 &&
					tmpPoint.getZ() >= -sizeZ/2 && tmpPoint.getZ() <= sizeZ/2) {
				resultPointCloud->addPoint(tmpPoint);
			}
		}
		delete inverseOrigin;
	}
}

}  // namespace BRICS_3D
/* EOF */
