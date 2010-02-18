/* OpenSceneGraph example, osgspacewarp.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */


#include <iostream>
#include <util/DepthImageLoader.h>
#include <util/OSGPointCloudVisualizer.h>
#include <core/PointCloud3D.h>
#include <algorithm/DepthImageToPointCloudTransformation.h>
#include <algorithm/filter/Octree.h>

using namespace std;
using namespace BRICS_3D;



int main(int argc, char **argv) {

	/* check argument */
	string filename;
	if (argc == 1) {
		cout << "Usage: " << argv[0] << " <filename>" << endl;

		char defaultFilename[255] = { BRICS_IMGAGES_DIR };
		strcat(defaultFilename, "/zcam_param1c.pgm\0");
		filename = defaultFilename;

		cout << "Trying to get default file: " << filename << endl;
	} else if (argc == 2) {
		filename = argv[1];
		cout << filename << endl;
	} else {
		cerr << "Usage: " << argv[0] << " <filename>" << endl;
		return -1;
	}

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

	/* (optionally) reduce  point cloud with octree filter */
	Octree* octreeFilter = new Octree();
	octreeFilter->setVoxelSize(5.0); //value deduce from roughly knowing the bounding box
	PointCloud3D* reducedPointCloud = new PointCloud3D();
	octreeFilter->createOctree(pointCloud, reducedPointCloud);

	/* visualize point cloud */
	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
//	visualizer->visualizePointCloud(pointCloud);
	visualizer->visualizePointCloud(reducedPointCloud);

	delete visualizer;
	delete reducedPointCloud;
	delete pointCloud;
	delete depthImgageLoader;

	return 0;
}

