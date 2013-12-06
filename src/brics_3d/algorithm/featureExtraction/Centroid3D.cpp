/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
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

#include "Centroid3D.h"
#include "brics_3d/core/Logger.h"

namespace brics_3d {

Centroid3D::Centroid3D() {

}

Centroid3D::~Centroid3D() {

}


Eigen::Vector3d Centroid3D::computeCentroid(brics_3d::PointCloud3D *inCloud){
	Eigen::Vector3d centroid;
	double tempX;
	double tempY;
	double tempZ;
	int count = 0;

	centroid[0] = 0;
	centroid[1] = 0;
	centroid[2] = 0;

	for (unsigned int i = 0; i < inCloud->getSize(); i++){
		tempX = (*inCloud->getPointCloud())[i].getX();
		tempY = (*inCloud->getPointCloud())[i].getY();
		tempZ = (*inCloud->getPointCloud())[i].getZ();

		if(!isnan(tempX) && !isinf(tempX) && !isnan(tempY) && !isinf(tempY) &&
				!isnan(tempZ) && !isinf(tempZ) ) {
			centroid[0] = centroid[0] + tempX;
			centroid[1] = centroid[1] + tempY;
			centroid[2] = centroid[2] + tempZ;
			count++;
		}
	}

	if (count == 0) {
		centroid[0] = 0;
		centroid[1] = 0;
		centroid[2] = 0;
		LOG(WARNING) << "Centroid3D: point cloud is empty. Returning (0,0,0).";
	} else {
		centroid[0] = centroid[0] / count;
		centroid[1] = centroid[1] / count;
		centroid[2] = centroid[2] / count;
	}

	return centroid;
}

Eigen::Vector3d Centroid3D::computeCentroid(IPoint3DIterator::IPoint3DIteratorPtr inCloud){
	Eigen::Vector3d centroid;
	double tempX;
	double tempY;
	double tempZ;
	int count = 0;

	centroid[0] = 0;
	centroid[1] = 0;
	centroid[2] = 0;

	for (inCloud->begin(); !inCloud->end(); inCloud->next()) {
		tempX = inCloud->getX();
		tempY = inCloud->getY();
		tempZ = inCloud->getZ();

		if(!isnan(tempX) && !isinf(tempX) && !isnan(tempY) && !isinf(tempY) &&
				!isnan(tempZ) && !isinf(tempZ) ) {
			centroid[0] = centroid[0] + tempX;
			centroid[1] = centroid[1] + tempY;
			centroid[2] = centroid[2] + tempZ;
			count++;
		}
	}

	if (count == 0) {
		centroid[0] = 0;
		centroid[1] = 0;
		centroid[2] = 0;
		LOG(WARNING) << "Centroid3D: point cloud is empty. Returning (0,0,0).";
	} else {
		centroid[0] = centroid[0] / count;
		centroid[1] = centroid[1] / count;
		centroid[2] = centroid[2] / count;
	}

	return centroid;
}

}
