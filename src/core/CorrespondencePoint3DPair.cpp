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

#include "CorrespondencePoint3DPair.h"

namespace brics_3d {

CorrespondencePoint3DPair::CorrespondencePoint3DPair() {

}

CorrespondencePoint3DPair::CorrespondencePoint3DPair(Point3D firstPoint, Point3D secondPoint) {
	this->firstPoint = Point3D(firstPoint);
	this->secondPoint = Point3D(secondPoint);
}

CorrespondencePoint3DPair::~CorrespondencePoint3DPair() {

}

}

/* EOF */
