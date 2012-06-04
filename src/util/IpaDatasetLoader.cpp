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

#include "IpaDatasetLoader.h"
#include "core/ColoredPoint3D.h"

namespace BRICS_3D {

IpaDatasetLoader::IpaDatasetLoader() {
	imageSize = cvSize(-1, -1);

	xyzImage = 0;
	colorImage = 0;

	xyzExtension = "_coloredPC_xyz_32F3.xml";
	zExtensionPNG = "_coloredPC_xyz_8U3.png";
	colorExtension = "_coloredPC_color_8U3.png";
}

IpaDatasetLoader::~IpaDatasetLoader() {
	release();
}

void IpaDatasetLoader::release(void) {
	if (xyzImage) {
		cvReleaseImage(&xyzImage);
		xyzImage = 0;
	}
	if (colorImage) {
		cvReleaseImage(&colorImage);
		colorImage = 0;
	}
}

bool IpaDatasetLoader::loadColoredPointCloud(std::string pathAndName) {
	release();
	setlocale(LC_NUMERIC, "C");
	std::string imageFileName;

	imageFileName = pathAndName + xyzExtension; // i.e. "name_xyz_Coord.xml";
	xyzImage = (IplImage*)cvLoad(imageFileName.c_str());
	if (xyzImage == 0)
	{
		LOG(ERROR) << "ColoredPointCloud::LoadColoredPointCloud: Could not load xyz image " << imageFileName;
		return false;
	}

	imageFileName = pathAndName + colorExtension; // i.e. "name_xyz_Shared.png";
	colorImage = cvLoadImage(imageFileName.c_str());
	if (colorImage == 0)
	{
		std::cerr << "ColoredPointCloud::LoadColoredPointCloud:" << std::endl;
		std::cerr << "\t ...  Could not load color image" << std::endl;
		std::cerr << "\t ... '" << imageFileName << "'\n";
		return false;
	}

	imageSize = cvSize(colorImage->width, colorImage->height);

	return true;
}



IplImage* IpaDatasetLoader::getXYZImage() {
	return xyzImage;
}

IplImage* IpaDatasetLoader::getColorImage() {
	return colorImage;
}

bool IpaDatasetLoader::getData(int i, int j, double& x, double& y, double& z, unsigned char& R, unsigned char& G, unsigned char& B) {
	if (xyzImage == NULL || colorImage == NULL) {
		LOG(ERROR) << "ColoredPointCloud::GetData: Color image or intensity image is a NULL-pointer";
		return false;
	}

	if (i >= xyzImage->width ||
			i >= colorImage->width ||
			i < 0 ||
			j >= xyzImage->height ||
			j >= colorImage->height ||
			j < 0) {

		LOG(ERROR) << "ColoredPointCloud::GetData: Parameter of color image or intensity image is out of bounds";
		return false;
	}

	float* ptrXYZ = &((float*) (xyzImage->imageData + j * xyzImage->widthStep))[i * 3];
	x = (double) ptrXYZ[0];
	y = (double) ptrXYZ[1];
	z = (double) ptrXYZ[2];

	unsigned char* ptr = &((unsigned char*) (colorImage->imageData + j * colorImage->widthStep))[i * 3];
	B = ptr[0];
	G = ptr[1];
	R = ptr[2];

	return true;
}

PointCloud3D* IpaDatasetLoader::getPointCloud() {

	PointCloud3D* pointCloud = new PointCloud3D();

	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	unsigned char red;
	unsigned char green;
	unsigned char blue;


	/* loop over all pixels and add those who are above a certain threshold */
	for (int row = 0; row < xyzImage->height; ++row) {
		for (int col = 0; col < xyzImage->width; ++col) {
			this->getData(row, col, x, y, z, red, green, blue);

			if (!((red == 0) && (green == 0) && (blue == 0))) { //discard "black" points as they don't belong to the object itself
				pointCloud->addPoint(Point3D(x, y, z));
			}
		}
	}

	/* plausibility check */
	assert(pointCloud->getSize() >= 0u && pointCloud->getSize() <= static_cast<unsigned int>(xyzImage->imageSize));

	return pointCloud;
}

PointCloud3D* IpaDatasetLoader::getColoredPointCloud() {

	PointCloud3D* pointCloud = new PointCloud3D();

	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	unsigned char red;
	unsigned char green;
	unsigned char blue;


	/* loop over all pixels and add those who are above a certain threshold */
	for (int row = 0; row < xyzImage->height; ++row) {
		for (int col = 0; col < xyzImage->width; ++col) {
			this->getData(row, col, x, y, z, red, green, blue);

			if (!((red == 0) && (green == 0) && (blue == 0))) { //discard "black" points as they don't belong to the object itself
				pointCloud->addPointPtr(new ColoredPoint3D(new Point3D(x, y, z), red, green, blue));
			}
		}
	}

	/* plausibility check */
	assert(pointCloud->getSize() >= 0u && pointCloud->getSize() <= static_cast<unsigned int>(xyzImage->imageSize));

	return pointCloud;
}

}

/* EOF */
