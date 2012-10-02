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
 * depth_image_transformation.cpp
 *
 * @brief Simple test file for conversion from depth image to point cloud
 *
 * @author: Sebastian Blumenthal
 * @date: Aug 20, 2009
 * @version: 0.1
 */


#include <iostream>
#include <brics_3d/util/DepthImageLoader.h>
#include <brics_3d/core/PointCloud3D.h>
#include <brics_3d/algorithm/depthPerception/DepthImageToPointCloudTransformation.h>

using namespace std;
using namespace brics_3d;


int main(int argc, char **argv) {

    /* check argument */
    if(argc != 2) {
        cerr << "Usage: " << argv[0] << " <filename>" << endl;
        cerr << "Try for example: " << argv[0] << " ../examples/test_data/depth_images/zcam_param1c.pgm" << endl;
        return -1;
    }
    string filename = argv[1];
    cout << filename << endl;

    /* get depth image*/
    IplImage* depthImage;
	DepthImageLoader *depthImgageLoader = new DepthImageLoader();
	depthImage = depthImgageLoader->loadDepthImage(filename);
	depthImgageLoader->displayDepthImage();

	/* convert to point cloud */
	PointCloud3D *pointCloud = new PointCloud3D();
	DepthImageToPointCloudTransformation *img2cloudTramsformer = new DepthImageToPointCloudTransformation();
	img2cloudTramsformer->transformDepthImageToPointCloud(depthImage, pointCloud, 0);
	cout << "Size of point cloud: " << pointCloud->getSize() << endl;

	/* store results */
	pointCloud->storeToTxtFile("test_point_cloud.txt");
	pointCloud->storeToPlyFile("test_point_cloud.ply");

	return 0;

}
