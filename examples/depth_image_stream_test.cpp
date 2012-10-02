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

/**
 * @file
 * depth_image_stream_test.cpp
 * 
 * @brief Simple test file to demonstrate depth image capturing from a video
 * stream
 *
 * @author: Sebastian Blumenthal
 * @date: Sep 15, 2009
 * @version: 0.1
 */

#include <iostream>
#include <brics_3d/util/DepthImageLoader.h>

using namespace std;
using namespace brics_3d;

int main(int argc, char **argv) {

    /* check argument */
    if(argc != 2) {
        cerr << "Usage: " << argv[0] << " <filename>" << endl;
        cerr << "Try for example: " << argv[0] << " ../examples/test_data/depth_images/zcam_stream.avi" << endl;
        return -1;
    }
    string filename = argv[1];
    cout << filename << endl;

    /* get depth images */
    IplImage* depthImage;
	DepthImageLoader *depthImageLoader = new DepthImageLoader();
	depthImage = depthImageLoader->loadDepthImageVideo(filename);

	/* create a window */
	cvNamedWindow("depthImageVideo", CV_WINDOW_AUTOSIZE);

	int i = 0;
	while(true) {
		cout << "Processing Image "<< ++i << endl;

		/* display the current image */
		cvShowImage("depthImageVideo", depthImage);

		/* wait a certain amount of time */
	    cvWaitKey(100);

		depthImage = depthImageLoader->getNextDepthImage();
		if (depthImage == NULL) {
			cout << endl << "Done" << endl;
			break;
		}
	}
	/* free memory */
	cvDestroyWindow("depthImageVideo");

}
/* EOF */
