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
#include "HomogeneousMatrix44.h"
#include "Logger.h"

namespace brics_3d {

PointCloud3DIterator::PointCloud3DIterator() {
	begin();
}

PointCloud3DIterator::~PointCloud3DIterator() {

}

std::string PointCloud3DIterator::getPointCloudTypeName() {
	return "brics_3d::PointCloud3D";
}

void PointCloud3DIterator::begin() {
	index = 0;
	pointCloudsIterator = pointCloudsWithTransforms.begin();
	associatedTransformIsIdentityIterator = associatedTransformIsIdentity.begin();

	while(!end() && (pointCloudsIterator->first->getSize() <= 0)) { //skip empty clouds
		pointCloudsIterator++;
		associatedTransformIsIdentityIterator++;
		LOG(WARNING) << "PointCloud3DIterator contains empty point clouds.";
	}

	if ( !end() ) {

		Point3D* tmpHandle = &((*pointCloudsIterator->first->getPointCloud())[index]); //only one operator[] access - which is sightly faster
		currentTransformedPoint.setX(tmpHandle->getX());
		currentTransformedPoint.setY(tmpHandle->getY());
		currentTransformedPoint.setZ(tmpHandle->getZ());
		if(*associatedTransformIsIdentityIterator == false) { // the non "lazyness" case
			currentTransformedPoint.homogeneousTransformation(pointCloudsIterator->second.get());
		}

	} else {
		pointCloudsIterator = pointCloudsWithTransforms.end(); // empty iterator
	}
}

void PointCloud3DIterator::next() {
	++index;

	if ( !end() ) {

		if(index >= pointCloudsIterator->first->getSize()) { //wrap over - advance to next point cloud
			index = 0;
			pointCloudsIterator++;
			associatedTransformIsIdentityIterator++;

			while(!end() && (pointCloudsIterator->first->getSize() <= 0)) { //skip further empty clouds
				pointCloudsIterator++;
				associatedTransformIsIdentityIterator++;
				LOG(WARNING) << "PointCloud3DIterator contains empty point clouds.";
			}
		}

		if ( !end() ) { // end could be reach meanwhile so we have to check again

			Point3D* tmpHandle = &((*pointCloudsIterator->first->getPointCloud())[index]); //only one operator[] access - which is slightly faster
			currentTransformedPoint.setX(tmpHandle->getX());
			currentTransformedPoint.setY(tmpHandle->getY());
			currentTransformedPoint.setZ(tmpHandle->getZ());
			if(*associatedTransformIsIdentityIterator == false) { // the non "lazyness" case
				currentTransformedPoint.homogeneousTransformation(pointCloudsIterator->second.get());
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
	return currentTransformedPoint.getX();
}

Coordinate PointCloud3DIterator::getY(){
	return currentTransformedPoint.getY();
}

Coordinate PointCloud3DIterator::getZ() {
	return currentTransformedPoint.getZ();
}

Point3D* PointCloud3DIterator::getRawData() {
	return &(*pointCloudsIterator->first->getPointCloud())[index];
}

void PointCloud3DIterator::insert(PointCloud3D::PointCloud3DPtr pointCloud, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr associatedTransform) {
	assert(pointCloud != 0);
	assert(associatedTransform != 0);
	pointCloudsWithTransforms.insert(std::make_pair(pointCloud, associatedTransform));
	associatedTransformIsIdentity.push_back(associatedTransform->isIdentity());
}

void PointCloud3DIterator::insert(PointCloud3D::PointCloud3DPtr pointCloud) {
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr identityTransform(new brics_3d::HomogeneousMatrix44());
	insert(pointCloud, identityTransform);
}

}

/* EOF */
