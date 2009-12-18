/**
 * @file 
 * IpaDatasetLoader.cpp
 *
 * @date: Dec 18, 2009
 * @author: sblume
 */

#include "IpaDatasetLoader.h"

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
		std::cerr << "ERROR - ColoredPointCloud::LoadColoredPointCloud:" << std::endl;
		std::cerr << "\t ... Could not load xyz image" << std::endl;
		std::cerr << "\t ... '" << imageFileName << "'\n";
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
		std::cerr << "ERROR - ColoredPointCloud::GetData:" << std::endl;
		std::cerr << "\t ... Color image or intensity image is a NULL-pointer'\n";
		return false;
	}

	if (i >= xyzImage->width ||
			i >= colorImage->width ||
			i < 0 ||
			j >= xyzImage->height ||
			j >= colorImage->height ||
			j < 0) {
		std::cerr << "ERROR - ColoredPointCloud::GetData:" << std::endl;
		std::cerr << "\t ... Parameter of color image or intensity image is aout of bounds'\n";
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
				pointCloud->addPoint(Point3D(x,y,z));
			}
		}
	}

	/* plausibility check */
	assert(pointCloud->getSize() >= 0u && pointCloud->getSize() <= static_cast<unsigned int>(xyzImage->imageSize));

	return pointCloud;
}

}

/* EOF */
