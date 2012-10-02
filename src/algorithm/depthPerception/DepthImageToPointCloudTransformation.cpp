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

#include "DepthImageToPointCloudTransformation.h"
#include <iostream>
#include <assert.h>
#include <stdio.h>

using namespace std;

namespace brics_3d {

DepthImageToPointCloudTransformation::DepthImageToPointCloudTransformation() {

}

DepthImageToPointCloudTransformation::~DepthImageToPointCloudTransformation() {

}

void DepthImageToPointCloudTransformation::transformDepthImageToPointCloud(IplImage *depthImage,
		PointCloud3D *pointCloud, double threshold) {

	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	CvScalar pixel;
	double pixelValue = 0.0;

	/* check if input parameters are valid */
	if (depthImage == NULL || pointCloud == NULL) {
		cerr << "ERROR: NULL pointer in input parameters" << endl;
		return;
	}

	/* loop over all pixels and add those who are above a certain threshold */
	for (int row = 0; row < depthImage->height; ++row) {
		for (int col = 0; col < depthImage->width; ++col) {

			/* access pixel in image */
			pixel = cvGet2D(depthImage, row, col);
			pixelValue = pixel.val[0]; // assumption: input is gray scale
			//cout << "pixelValue: " << pixelValue /*<< endl*/; //DBG

			/* faster variant: (but should be more tested before usage) */
			//double pixelValue_test = (double)(uchar)*(depthImage->imageData + row*depthImage->widthStep + col);
			//cout << " pixelValue_test: " << pixelValue_test << endl; //DBG
			//assert(pixelValue == pixelValue_test);

			if (pixelValue >= threshold) {
				/* TODO is now uncalibrated */
				//x = col;
				//y = row;
				//z = pixelValue;
				x = col;
				assert((0.0 <= pixelValue) && (pixelValue <= 255.0)); //comment out for optimization
				y = MAX_DEPTHIMAGE_VALUE - pixelValue; // invert here because bright regions appears nearer (at least for zcam) TODO: check if this common
				z = depthImage->height - row; //flips the image (because of negative y axis definition in depth images)
				pointCloud->addPoint(Point3D(x,y,z));
			}
		}
	}

	/* plausibility check */
	assert(pointCloud->getSize() >= 0u && pointCloud->getSize() <= static_cast<unsigned int>(depthImage->imageSize));
}

}

/* EOF */
