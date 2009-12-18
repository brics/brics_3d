/**
 * @file 
 * IpaDatasetLoader.h
 *
 * @date: Dec 18, 2009
 * @author: sblume
 */

#ifndef IPADATASETLOADER_H_
#define IPADATASETLOADER_H_

#include "core/PointCloud3D.h"

#ifdef WIN32
	#include <windows.h>
#endif

#include <string>
#include <iostream>
#include <cv.h>
#include <highgui.h>


namespace BRICS_3D {

/**
 * @brief Helper class to load x, y, z and RGB data from IPA like data sets.
 */
class IpaDatasetLoader {
public:

	/**
	 * @brief Standard constructor
	 */
	IpaDatasetLoader();

	/**
	 * @brief Standard destructor
	 */
	virtual ~IpaDatasetLoader();

	/**
	 * @brief Constructor
	 * @param imageSize Size of shared image
	 */
	IpaDatasetLoader(const CvSize& imageSize);

	/**
	 * @brief Releases images if allocated.
	 * Images are freed with <code>cvReleaseImage</code> and set to point to NULL.
	 */
	void  release();

	/**
	 * @brief Loads the xyz and color image from the file specified by <code> pathAndName </code>
	 * @param pathAndName Prefix for the actual filename (i.e. 'pathAndName'_Coord.xml).
	 * @return True if all images could be loaded, false otherwise.
	 */
 	bool loadColoredPointCloud(std::string pathAndName);

 	/**
 	 * Returns pointer to the xyz image.
	 * The xyz image holds one corresponding x,y,z-coordinate triple for each image coordinate
	 * @return The cartesian coordinate image.
	 */
	IplImage* getXYZImage();

	/**
	 * Returns pointer to the color image.
	 * The color image holds the color information of the 3D data
	 * @return The shared image.
	 */
	IplImage* getColorImage();

	/**
	 * Returns to a pair of image coordinates the corresponding color and depth values.
	 * @param i x-coordinate of image	//bool m_initialized;
	 * @param j y-coordinate of image
	 * @param x Corresponding x-coordinate of (i,j)-image-pixel
	 * @param y Corresponding y-coordinate of (i,j)-image-pixel
	 * @param z Corresponding z-coordinate of (i,j)-image-pixel
	 * @param R Corresponding red color component of (i,j)-image-pixel
	 * @param G Corresponding green color component of (i,j)-image-pixel
	 * @param B Corresponding blue color component of (i,j)-image-pixel
	 * @return True if method succesfully finished, false if an error occurred.
	 */
	bool getData(int i, int j, double& x, double& y, double& z, unsigned char& R, unsigned char& G, unsigned char& B);

	/**
	 * @brief Get a <code>PointCloud3D</code> representation of the data.
	 */
	PointCloud3D* getPointCloud();

private:

	/// The 3D coordinates
	IplImage* xyzImage;

	/// The color values corresponding to the 3D coordinates
	IplImage* colorImage;

	/// The common size of the xyz image and the color image
	CvSize imageSize;


	/// Extension of filename, when saving 32 bit, 3 channel xyz image
	std::string xyzExtension;

	/// Extension of filename, when saving 8 bit, 3 channel z image
	std::string zExtensionPNG;

	/// Extension of filename, when saving 8 bit, 3 channel rgb shared image
	std::string colorExtension;

};

}

#endif /* IPADATASETLOADER_H_ */

/* EOF */
