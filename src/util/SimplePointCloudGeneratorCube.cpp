/*
 * SimplePointCloudGeneratorCube.cpp
 *
 *  Created on: Oct 5, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "SimplePointCloudGeneratorCube.h"

namespace BRICS_3D {


SimplePointCloudGeneratorCube::SimplePointCloudGeneratorCube() {

	setCubeSideLength(1);
	setNumOfFaces(3);

	double origin[3];
	origin[0]=0; origin[1]=0; origin[2]=0;
	setOrigin(origin);

	setPointsOnEachSide(10);
}


double SimplePointCloudGeneratorCube::getCubeSideLength() const
{
	return cubeSideLength;
}


int SimplePointCloudGeneratorCube::getNumOfFaces() const
{
	return numOfFaces;
}


int SimplePointCloudGeneratorCube::getPointsOnEachSide() const
{
	return pointsOnEachSide;
}


void SimplePointCloudGeneratorCube::setCubeSideLength(double cubeSideLength)
{
	this->cubeSideLength = cubeSideLength;
}


void SimplePointCloudGeneratorCube::setNumOfFaces(int numOfFaces)
{
	if(numOfFaces<=3){
		this->numOfFaces = numOfFaces;
	} else {
		this->numOfFaces = 3;
	}
}


void SimplePointCloudGeneratorCube::setOrigin(double origin[3])
{
	this->origin[0] = origin[0];
	this->origin[1] = origin[1];
	this->origin[2] = origin[2];
}


void SimplePointCloudGeneratorCube::setPointsOnEachSide(int pointsOnEachSide)
{
	this->pointsOnEachSide = pointsOnEachSide;
}


void SimplePointCloudGeneratorCube::getOrigin(double *x, double *y, double *z){
	*x = this->origin[0];
	*y = this->origin[1];
	*z = this->origin[2];
}


SimplePointCloudGeneratorCube::~SimplePointCloudGeneratorCube() {
	// TODO Auto-generated destructor stub
}


void SimplePointCloudGeneratorCube::generatePointCloud(BRICS_3D::PointCloud3D *generatedPointCloud){

	double incr = (this->cubeSideLength)/ ((double)(this->pointsOnEachSide));
	float xincr, yincr, zincr;
	BRICS_3D::Point3D tempPoint3D;

	generatedPointCloud->getPointCloud()->clear();

    origin[0]= origin[0] -cubeSideLength/2.0;
    origin[1]= origin[1] -cubeSideLength/2.0;
    origin[2]= origin[2] -cubeSideLength/2.0;



	for (int face=0; face < numOfFaces; face++) {
		switch(face){

		case 0:
			//xy-plane
			xincr = origin[0];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < pointsOnEachSide; i++){
				yincr = origin[1];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < pointsOnEachSide; j++) {
					tempPoint3D.setX(xincr);
					tempPoint3D.setY(yincr);
					tempPoint3D.setZ(origin[2]+cubeSideLength);
					generatedPointCloud->addPoint(tempPoint3D);
					yincr += incr;
				}
				xincr += incr;
			}
			break;

		case 1:
			//yz-plane
			yincr = origin[1];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < pointsOnEachSide; i++){
				zincr = origin[2];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < pointsOnEachSide; j++) {
					tempPoint3D.setX(origin[0]);
					tempPoint3D.setY(yincr);
					tempPoint3D.setZ(zincr);
					generatedPointCloud->addPoint(tempPoint3D);
					zincr += incr;
				}
				yincr += incr;
			}
			break;

		case 2:
			//xz-plane
			xincr = origin[0];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < pointsOnEachSide; i++){
				zincr = origin[2];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < pointsOnEachSide; j++) {
					tempPoint3D.setX(xincr);
					tempPoint3D.setY(origin[1]);
					tempPoint3D.setZ(zincr);
					generatedPointCloud->addPoint(tempPoint3D);
					zincr += incr;
				}
				xincr += incr;
			}

			break;

		default:
			break;
		}
	}

/*	Eigen::Matrix4f  tempHomogenousMatrix;
	calculateHomogeneousMatrix(90,0,0,0,0,0,tempHomogenousMatrix,true);
    applyHomogeneousTransformation(&cubeModel3D, &cubeModel3D, tempHomogenousMatrix);
    applyHomogeneousTransformation(&cubeModel3D, &cubeModel3D, tempHomogenousMatrix);
*/
}


}
