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
#include <util/DepthImageLoader.h>
#include <core/PointCloud3D.h>
#include <algorithm/depthPerception/DepthImageToPointCloudTransformation.h>

using namespace std;
using namespace BRICS_3D;


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
