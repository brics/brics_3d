/*
 * Centroid3D.cpp
 *
 *  Created on: Oct 17, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "Centroid3D.h"

namespace BRICS_3D {

Centroid3D::Centroid3D() {
	// TODO Auto-generated constructor stub

}

Centroid3D::~Centroid3D() {
	// TODO Auto-generated destructor stub
}


Eigen::Vector3d Centroid3D::computeCentroid(BRICS_3D::PointCloud3D *inCloud){
	Eigen::Vector3d centroid;
	double tempX, tempY, tempZ;
	int count =0;
	for(unsigned int i = 0; i<inCloud->getSize();i++){
		tempX = inCloud->getPointCloud()->data()[i].getX();
		tempY = inCloud->getPointCloud()->data()[i].getY();
		tempZ = inCloud->getPointCloud()->data()[i].getZ();

		if(!isnan(tempX) && !isinf(tempX) && !isnan(tempY) && !isinf(tempY) &&
				!isnan(tempZ) && !isinf(tempZ) ) {
			centroid[0]+= tempX;
			centroid[1]+= tempY;
			centroid[2]+= tempZ;
			count++;
		}
	}

	centroid[0] /= count;
	centroid[1] /= count;
	centroid[2] /= count;

	return centroid;
}


Eigen::Vector3d Centroid3D::computeCentroid(BRICS_3D::ColoredPointCloud3D *inCloud){
	Eigen::Vector3d centroid;
	double tempX, tempY, tempZ;
	int count =0;
	for(unsigned int i = 0; i<inCloud->getSize();i++){
		tempX = inCloud->getPointCloud()->data()[i].getX();
		tempY = inCloud->getPointCloud()->data()[i].getY();
		tempZ = inCloud->getPointCloud()->data()[i].getZ();

		if(!isnan(tempX) && !isinf(tempX) && !isnan(tempY) && !isinf(tempY) &&
				!isnan(tempZ) && !isinf(tempZ) ) {
			centroid[0]+= tempX;
			centroid[1]+= tempY;
			centroid[2]+= tempZ;
			count++;
		}
	}

	centroid[0] /= count;
	centroid[1] /= count;
	centroid[2] /= count;

	return centroid;
}


}
