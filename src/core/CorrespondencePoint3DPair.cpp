/**
 * @file 
 * CorrespondencePoint3DPair.cpp
 *
 * @date: Dec 7, 2009
 * @author: sblume
 */

#include "CorrespondencePoint3DPair.h"

namespace BRICS_3D {

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
