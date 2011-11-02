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

#include "DepthImageLoader.h"

namespace BRICS_3D {

DepthImageLoader::DepthImageLoader() {
	image = NULL;
	video = NULL;
}

DepthImageLoader::~DepthImageLoader() {
	cvReleaseImage(&image);
	cvReleaseCapture(&video);
	if (image)
		delete image;
	//if (video)
	//	delete video;
}

IplImage* DepthImageLoader::loadDepthImage(string filename) {

	/* load the image */
	image = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

	/* check for errors */
	if (image == NULL) {
		cerr << "ERROR: Cannot load image file: " << filename << endl;
	}

	/* check if input image is only in gray scale */
	assert(image->nChannels == 1); //only brightness channel
	assert(image->depth == IPL_DEPTH_8U); // 8bit depth => uchar => range is 0-255

	return image;
}

IplImage* DepthImageLoader::loadDepthImageVideo(string filename) {

	/* initialize video stream */
	video = cvCaptureFromFile(filename.c_str());
    if(video==NULL) {
        cerr << "ERROR: Cannot open video file: " << filename << endl;
        cerr << "This error might occur if the code is running on a debian likes system with OpenCV installed from the repository. ";
        cerr << "Please download the source code of OpenCV, and make sure ffmpeg is supported when compiling it."  << endl;
        return NULL;
    }

    /* retrieve first image */
    image = cvQueryFrame(video);

    /* in case of non gray level videos */
    if (image->nChannels > 1) {
    	cerr << "WARNING: video stream might contain RGB data!" << endl;
		CvScalar pixel;

		/* check if all channels are equal */
		for (int row = 0; row < image->height; ++row) {
			for (int col = 0; col < image->width; ++col) {
				pixel = cvGet2D(image, row, col);
				switch (image->nChannels) {
					case 2:
						assert(pixel.val[0] == pixel.val[1]);
						break;
					case 3:
						assert((pixel.val[0] == pixel.val[1]) && (pixel.val[1] == pixel.val[2]));
						break;
					case 4:
						assert((pixel.val[0] == pixel.val[1]) && (pixel.val[1] == pixel.val[2]) && (pixel.val[2] == pixel.val[3]));
						break;
					default:
						cerr << "ERROR: video stream contains too many channels" << endl;
							return NULL;
				}
			}
		}
		cerr << "But all channels are equal (at least in the first frame). Treating this as gray value stream." << endl;
    }
	return image;
}

IplImage* DepthImageLoader::getDepthImage() {

	/* check if image is loaded */
	if (image == NULL) {
		cerr << "WARNING: No image available. Please load one first one." << endl;
	}

	return image;
}

IplImage* DepthImageLoader::getNextDepthImage() {

	/* check if image and video are loaded */
	if (image == NULL || video == NULL) {
		cerr << "WARNING: No video stream available. Please load one first one." << endl;
	}

    /* retrieve next image */
    image = cvQueryFrame(video);

    return image;
}

void DepthImageLoader::displayDepthImage() {

	/* check if image is loaded */
	if (image == NULL) {
		cerr << "WARNING: No image available. Please load one first."
				<< endl;
		return;
	}

	/* create a window */
	cvNamedWindow("depthImage", CV_WINDOW_AUTOSIZE);

	/* display the image */
	cvShowImage("depthImage", image);

	/* wait until user press a key */
	cvWaitKey(0);

	/* free memory */
	cvDestroyWindow("depthImage");

}

}
