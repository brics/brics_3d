/*
 * ColorBasedROIExtractorRGB.cpp
 *
 *  Created on: Dec 2, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "ColorBasedROIExtractorRGB.h"

namespace BRICS_3D {

ColorBasedROIExtractorRGB::ColorBasedROIExtractorRGB() {

	this->red = 0;
	this->green = 0;
	this->blue = 0;
	this->distanceThresholdMinimum = 0.0;
	this->distanceThresholdMaximum = 0.0;
}

ColorBasedROIExtractorRGB::~ColorBasedROIExtractorRGB() {
	// TODO Auto-generated destructor stub
}

void ColorBasedROIExtractorRGB::extractColorBasedROI(BRICS_3D::ColoredPointCloud3D *in_cloud, BRICS_3D::ColoredPointCloud3D *out_cloud){

	if(this->red == 0 && this->green == 0 && this->blue == 0 ) {
		printf("[WARNING] Using limits: R=0, G=0, B=0 for RGB based ROI Extraction!!!\n");
	}

	unsigned int cloudSize =	in_cloud->getSize();
	int tempR, tempG, tempB;
	uint8_t tempChar;
	bool passed;
	BRICS_3D::Point3D tempPoint3D;
	out_cloud->getPointCloud()->clear();


	for (unsigned int i = 0; i < cloudSize; i++) {
		passed = false;
		//Getting the HSV values for the RGB points
		tempChar = in_cloud->getPointCloud()->data()[i].red;
		tempR = tempChar << 0;
		tempR = abs(tempR);

		tempChar = in_cloud->getPointCloud()->data()[i].green;
		tempG = tempChar << 0;
		tempG = abs(tempG);

		tempChar = in_cloud->getPointCloud()->data()[i].blue;
		tempB = tempChar << 0;
		tempB = abs(tempB);

//		printf("[INFO] Current R=%d, G=%d, B=%d, \n", tempR, tempG, tempB);

		double redDistance = (double)(tempR-this->red);
		double greenDistance = (double)(tempG-this->green);
		double blueDistance = (double)(tempB-this->blue);

		double currentDistance = (redDistance*redDistance) + (greenDistance*greenDistance) +
									(blueDistance*blueDistance);
		currentDistance = std::sqrt(currentDistance);

		if (currentDistance <= distanceThresholdMaximum && currentDistance >= distanceThresholdMinimum) passed=true;

		if(passed){
			BRICS_3D::Point3D *tempPoint3D =  new BRICS_3D::Point3D(in_cloud->getPointCloud()->data()[i].getX(),
					in_cloud->getPointCloud()->data()[i].getY(),
					in_cloud->getPointCloud()->data()[i].getZ());

			BRICS_3D::ColoredPoint3D *tempColoredPoint3D = new BRICS_3D::ColoredPoint3D(tempPoint3D,
					in_cloud->getPointCloud()->data()[i].red,
					in_cloud->getPointCloud()->data()[i].green,
					in_cloud->getPointCloud()->data()[i].blue);

			out_cloud->addPoint(tempColoredPoint3D);
			delete tempPoint3D;
			delete tempColoredPoint3D;
		}

	}


}


void ColorBasedROIExtractorRGB::extractColorBasedROI(BRICS_3D::ColoredPointCloud3D *in_cloud, BRICS_3D::PointCloud3D *out_cloud){

}
}
