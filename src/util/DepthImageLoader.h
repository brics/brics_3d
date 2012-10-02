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

#ifndef DEPTHIMAGELOADER_H_
#define DEPTHIMAGELOADER_H_

#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>

using namespace std;

/**
 * @namespace brics_3d
 */
namespace brics_3d {

/**
 * @class DepthImageLoader
 * @brief Class to load a depth image (from harddisk; uses OpenCV)
 * @author Sebastian Blumenthal
 * @date Aug 20, 2009
 */
class DepthImageLoader {
public:

	/**
	 * @brief Standard constuctor
	 */
	DepthImageLoader();

	/**
	 * @brief Standard destructor
	 */
	virtual ~DepthImageLoader();

	/**
	 * @brief Load a depth image
	 * @param filename Specifies the filename of the stored image.
	 * @return Returns the loaded depth image
	 */
	IplImage* loadDepthImage(string filename);

	/**
	 * @brief Load a depth image video stream
	 *
	 * Instead of a single image a video stream is loaded. Whenever the getNextDepthImage()
	 * function is invoked the next depth image is loaded and replaces the current one.
	 * Note: loadDepthImageVideo already returns the first depth image e.i. the first
	 * invocation of getNextDepthImage() returns the second image of the video stream.
	 *
	 * @param filename Specifies the filename of the stored video.
	 * @return Returns the first loaded depth image
	 */
	IplImage* loadDepthImageVideo(string filename);

	/**
	 * @brief Returns the last loaded depth image
	 */
	IplImage* getDepthImage();

	/**
	 * @brief Loads the next image (if available) and returns it
	 *
	 * If no further images are available NULL is returned.
	 */
	IplImage* getNextDepthImage();

	/**
	 * @brief Displays the last loaded depth image (in an OpenCV window)
	 */
	void displayDepthImage();

private:

	/// Reference that holds the loaded image
	IplImage* image;

	/// Reference that holds the video stream
	CvCapture* video;

};

}

#endif /* DEPTHIMAGELOADER_H_ */
