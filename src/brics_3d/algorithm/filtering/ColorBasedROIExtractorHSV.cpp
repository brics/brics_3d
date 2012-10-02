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

#include "ColorBasedROIExtractorHSV.h"

#include "brics_3d/core/ColorSpaceConvertor.h"
#include "brics_3d/core/ColoredPoint3D.h"
#include <stdio.h>

namespace brics_3d {

ColorBasedROIExtractorHSV::ColorBasedROIExtractorHSV() {
	this->maxH = 255;
	this->minH	= 0;

	this->maxS = 255;
	this->minS	= 0;

	this->maxV = 255;
	this->minV	= 0;

}

double ColorBasedROIExtractorHSV::getMaxH() const
{
	return maxH;
}

double ColorBasedROIExtractorHSV::getMaxS() const
{
	return maxS;
}

double ColorBasedROIExtractorHSV::getMaxV() const
{
	return maxV;
}

double ColorBasedROIExtractorHSV::getMinH() const
{
	return minH;
}

double ColorBasedROIExtractorHSV::getMinS() const
{
	return minS;
}

double ColorBasedROIExtractorHSV::getMinV() const
{
	return minV;
}

void ColorBasedROIExtractorHSV::setMaxH(double maxH)
{
	this->maxH = maxH;
}

void ColorBasedROIExtractorHSV::setMaxS(double maxS)
{
	this->maxS = maxS;
}

void ColorBasedROIExtractorHSV::setMaxV(double maxV)
{
	this->maxV = maxV;
}

void ColorBasedROIExtractorHSV::setMinH(double minH)
{
	this->minH = minH;
}

void ColorBasedROIExtractorHSV::setMinS(double minS)
{
	this->minS = minS;
}

void ColorBasedROIExtractorHSV::setMinV(double minV)
{
	this->minV = minV;
}

ColorBasedROIExtractorHSV::~ColorBasedROIExtractorHSV() {}

void ColorBasedROIExtractorHSV::filter(brics_3d::PointCloud3D* originalPointCloud,
		brics_3d::PointCloud3D* resultPointCloud){

	if(this->minS == 0 && this->minH == 0 && this->minV == 0 && this->maxH == 255 &&
			this->maxS == 255 && this->maxV == 255) {
		printf("[WARNING] Using maximum limits for HSV based ROI Extraction!!!\n");
	}

	int cloudSize =	originalPointCloud->getSize();
	double tempH, tempS, tempV;
	int tempR, tempG, tempB;
	uint8_t tempChar;
	bool passed;
	brics_3d::ColorSpaceConvertor colorConvertor;
	brics_3d::Point3D tempPoint3D;
	resultPointCloud->getPointCloud()->clear();

	printf("Used H-S Limits for extraction: H:[%f %f] S:[%f %f]\n", minH, maxH, minS, maxS);
	for (unsigned int i = 0; i < static_cast<unsigned int>(cloudSize); i++) {
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

		colorConvertor.rgbToHsv(tempR, tempG, tempB, &tempH, &tempS, &tempV);
		if(minH < maxH){


			//Checking the values with the set limits
			if (tempH <= maxH && tempH >= minH) {
				if (tempS >= minS && tempS <= maxS) {
					passed=true;

				}
			}


			//		printf("H-S Limits: [%f %f %f %f]\n", minH, maxH, minS, maxS);
			//		printf("Actual H-S Values: [%d %d %d %f %f]\n", tempR, tempG, tempB, tempH, tempS);
			if(passed){
				resultPointCloud->addPointPtr((*originalPointCloud->getPointCloud())[i].clone());
			}


		}else {

			//Checking the values with the set limits
			if ((tempH <= 255 && tempH >= minH) || (tempH >= 0 && tempH <= maxH)) {
				if (tempS >= minS && tempS <= maxS) {
					passed=true;

				}
			}


			//		printf("H-S Limits: [%f %f %f %f]\n", minH, maxH, minS, maxS);
			//		printf("Actual H-S Values: [%d %d %d %f %f]\n", tempR, tempG, tempB, tempH, tempS);
			if(passed){
				resultPointCloud->addPointPtr((*originalPointCloud->getPointCloud())[i].clone());
			}
		}
	}
	//	printf("Output cloud size:%d\n", out_cloud->getSize());
}




}
