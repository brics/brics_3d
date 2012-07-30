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


#include "PointCloud3DIterator.h"

namespace BRICS_3D {

PointCloud3DIterator::PointCloud3DIterator() {
	currentTransformedPoint = 0;
	begin();
}

PointCloud3DIterator::~PointCloud3DIterator() {

}

void PointCloud3DIterator::begin() {
	index = 0;
	pointCloudsIterator = pointCloudsWithTransforms.begin();

	if ( !end() ) {
		if (currentTransformedPoint) { //delete old copy
			delete currentTransformedPoint;
			currentTransformedPoint = 0;
		}

		/* cache the a transformed copy */
		currentTransformedPoint = (*pointCloudsIterator->first->getPointCloud())[index].clone();
		currentTransformedPoint->homogeneousTransformation(pointCloudsIterator->second);
	}
}

void PointCloud3DIterator::next() {
	++index;

	if ( !end() ) {
		if(index >= pointCloudsIterator->first->getSize()) { //wrap over - advance to next point cloud
			index = 0;
			pointCloudsIterator++;
		}

		if ( !end() ) { // end could be reach meanwhile so we have to check again
			if(false && pointCloudsIterator->second->isIdentity()) { // "lazyness" case
				/* cache the real data */
				currentTransformedPoint = &(*pointCloudsIterator->first->getPointCloud())[index];
			} else {
				if (currentTransformedPoint) { //delete old copy
					delete currentTransformedPoint;
					currentTransformedPoint = 0;
				}

				/* cache the a transformed copy */
				currentTransformedPoint = (*pointCloudsIterator->first->getPointCloud())[index].clone();
				currentTransformedPoint->homogeneousTransformation(pointCloudsIterator->second);
			}
		}
	} else {
		/* no further iterations, we are at the end */
	}
}

bool PointCloud3DIterator::end() {
	if (pointCloudsIterator ==  pointCloudsWithTransforms.end()) {
			return true;
	}
	return false;
}

Coordinate PointCloud3DIterator::getX() {
	return currentTransformedPoint->getX();
}

Coordinate PointCloud3DIterator::getY(){
	return currentTransformedPoint->getY();
}

Coordinate PointCloud3DIterator::getZ() {
	return currentTransformedPoint->getZ();
}

Point3D* PointCloud3DIterator::getRawData() {
	return &(*pointCloudsIterator->first->getPointCloud())[index];
}

//void PointCloud3DIterator::insert(PointCloud3D::PointCloud3DPtr pointCloud, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr associatedTransform) {
void PointCloud3DIterator::insert(PointCloud3D* pointCloud, IHomogeneousMatrix44* associatedTransform) {

	pointCloudsWithTransforms.insert(std::make_pair(pointCloud, associatedTransform));
}


}

/* EOF */
