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

#include "SimplePointCloudGeneratorCube.h"

namespace brics_3d {


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
	if(numOfFaces<=6){
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


void SimplePointCloudGeneratorCube::generatePointCloud(brics_3d::PointCloud3D *generatedPointCloud){

//	std::cout<< "Generating the cloud" << std::endl;

	double incr = (this->cubeSideLength)/ ((double)(this->pointsOnEachSide));
	float xincr, yincr, zincr;
	brics_3d::Point3D tempPoint3D;

//	generatedPointCloud->getPointCloud()->clear();
	double orgn[3];
    orgn[0]= this->origin[0]- this->cubeSideLength/2.0;
    orgn[1]= this->origin[0]- this->cubeSideLength/2.0;
    orgn[2]= this->origin[0]-this->cubeSideLength/2.0;



	for (int face=0; face < numOfFaces; face++) {
		switch(face){

		case 0:
			//xy-plane
			xincr = orgn[0];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < pointsOnEachSide; i++){
				yincr = orgn[1];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < pointsOnEachSide; j++) {
					tempPoint3D.setX(xincr);
					tempPoint3D.setY(yincr);
					tempPoint3D.setZ(orgn[2]+cubeSideLength);
					generatedPointCloud->addPoint(tempPoint3D);
					yincr += incr;
				}
				xincr += incr;
			}
			break;

		case 1:
			//yz-plane
			yincr = orgn[1];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < pointsOnEachSide; i++){
				zincr = orgn[2];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < pointsOnEachSide; j++) {
					tempPoint3D.setX(orgn[0]);
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
			xincr = orgn[0];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < pointsOnEachSide; i++){
				zincr = orgn[2];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < pointsOnEachSide; j++) {
					tempPoint3D.setX(xincr);
					tempPoint3D.setY(orgn[1]);
					tempPoint3D.setZ(zincr);
					generatedPointCloud->addPoint(tempPoint3D);
					zincr += incr;
				}
				xincr += incr;
			}

			break;

		case 3:

			xincr = orgn[0];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < pointsOnEachSide; i++){
				yincr = orgn[1];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < pointsOnEachSide; j++) {
					tempPoint3D.setX(xincr);
					tempPoint3D.setY(yincr);
					tempPoint3D.setZ(orgn[2]);
					generatedPointCloud->addPoint(tempPoint3D);
					yincr += incr;
				}
				xincr += incr;
			}
			break;

		case 4:
			//yz-plane
			yincr = orgn[1];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < pointsOnEachSide; i++){
				zincr = orgn[2];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < pointsOnEachSide; j++) {
					tempPoint3D.setX(orgn[0]+cubeSideLength);
					tempPoint3D.setY(yincr);
					tempPoint3D.setZ(zincr);
					generatedPointCloud->addPoint(tempPoint3D);
					zincr += incr;
				}
				yincr += incr;
			}
			break;

		case 5:
			//xz-plane
			xincr = orgn[0];//+maxWidthCube/cloudWidthEachFace;
			for (int i = 0; i < pointsOnEachSide; i++){
				zincr = orgn[2];//+maxWidthCube/cloudWidthEachFace;
				for (int j = 0; j < pointsOnEachSide; j++) {
					tempPoint3D.setX(xincr);
					tempPoint3D.setY(orgn[1]+cubeSideLength);
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
