/*
 * ColorBasedROIExtractorRGB.cpp
 *
 *  Created on: Dec 2, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "ColorBasedROIExtractorRGB.h"
#include "core/ColoredPoint3D.h"
#include "core/Logger.h"

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

void ColorBasedROIExtractorRGB::filter(PointCloud3D *originalPointCloud, PointCloud3D *resultPointCloud) {

	if(this->red == 0 && this->green == 0 && this->blue == 0 ) {
		LOG(WARNING) << "[WARNING] Using limits: R=0, G=0, B=0 for RGB based ROI Extraction!!!";
	}

	unsigned int cloudSize = originalPointCloud->getSize();
	int tempR, tempG, tempB;
	uint8_t tempChar;
	bool passed;
	BRICS_3D::Point3D tempPoint3D;
	resultPointCloud->getPointCloud()->clear();


	for (unsigned int i = 0; i < cloudSize; i++) {
		passed = false;

		if( (*originalPointCloud->getPointCloud())[i].asColoredPoint3D() == 0) {
			continue; //this point does not contain color information so skip it
		}

		//Getting the HSV values for the RGB points
		tempChar = (*originalPointCloud->getPointCloud())[i].asColoredPoint3D()->getR();
		tempR = tempChar << 0;
		tempR = abs(tempR);

		tempChar = (*originalPointCloud->getPointCloud())[i].asColoredPoint3D()->getG();
		tempG = tempChar << 0;
		tempG = abs(tempG);

		tempChar = (*originalPointCloud->getPointCloud())[i].asColoredPoint3D()->getB();
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
//			BRICS_3D::Point3D *tempPoint3D =  new BRICS_3D::Point3D(originalPointCloud->getPointCloud()->data()[i].getX(),
//					originalPointCloud->getPointCloud()->data()[i].getY(),
//					originalPointCloud->getPointCloud()->data()[i].getZ());
//
//			BRICS_3D::ColoredPoint3D *tempColoredPoint3D = new BRICS_3D::ColoredPoint3D(tempPoint3D,
//					originalPointCloud->getPointCloud()->data()[i].red,
//					originalPointCloud->getPointCloud()->data()[i].green,
//					originalPointCloud->getPointCloud()->data()[i].blue);
//
//			out_cloud->addPoint(tempColoredPoint3D);
			resultPointCloud->addPointPtr((*originalPointCloud->getPointCloud())[i].clone());
//			delete tempPoint3D;
//			delete tempColoredPoint3D;
		}

	}


}

}
