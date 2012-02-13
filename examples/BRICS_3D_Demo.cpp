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

/* BRICS_3D includes */
#include <util/DepthImageLoader.h>
#include <util/OSGPointCloudVisualizer.h>
#include <util/OSGTriangleMeshVisualizer.h>
#include <core/PointCloud3D.h>
#include <core/HomogeneousMatrix44.h>
#include <core/TriangleMeshImplicit.h>
#include <core/TriangleMeshExplicit.h>
#include <algorithm/filtering/Octree.h>
#include <algorithm/registration/IterativeClosestPointFactory.h>
#include <algorithm/depthPerception/DepthImageToPointCloudTransformation.h>
#include <algorithm/meshGeneration/DelaunayTriangulationOSG.h>

/* general includes */
#include <iostream>
#include <cstring>

using namespace std;
using namespace BRICS_3D;


/*
 * The program demonstrates some capabilities of the BRICS_3D library.
 * It will:
 *  1.) start by loading either a depth image or a point cloud,
 *  2.) down-sample the data in an Octree-based filtering step,
 *  3.) optionally register two point clouds into common coordinate system,
 *  4.) creates triangle mesh and
 *  5.) visualize the result.
 *
 */
int main(int argc, char **argv) {
	bool loadDepthImage = false;    // if true, load depth  image, other wise load from txt file
	int nInputClouds = 1;           // number of input point clouds

	/* check arguments
	 * In case no arguments are given a default depth image will be loaded.
	 * In case one argument is given it will be treated as a depth image.
	 * In case two arguments are given the files will be treated as txt files.
	 */
	string filename1;
	string filename2;
	if (argc == 1) {
		cout << "Usage: " << argv[0] << " <filename>" << endl;

		char defaultFilename[255] = { BRICS_IMGAGES_DIR };
		strcat(defaultFilename, "/zcam_param1c.pgm\0");
		filename1 = defaultFilename;

		cout << "Trying to get default file: " << filename1 << endl;
		loadDepthImage = true;
	} else if (argc == 2) {
		filename1 = argv[1];
		loadDepthImage = false;
		cout << filename1 << endl;
	} else if (argc == 3) {
		filename1 = argv[1];
		filename2 = argv[2];
		nInputClouds = 2;
		cout << filename1 << ", " << filename2 << endl;
	} else {
		cerr << "Usage: " << argv[0] << " <filename>" << endl;
		return -1;
	}

	/* create the point cloud containers */
	PointCloud3D* pointCloud1 = new PointCloud3D();
	PointCloud3D* pointCloud2 = new PointCloud3D();

	/* load 3d data */
	if (loadDepthImage == true) {

		/* get depth image */
		IplImage* depthImage;
		DepthImageLoader* depthImgageLoader = new DepthImageLoader();
		depthImage = depthImgageLoader->loadDepthImage(filename1);
		depthImgageLoader->displayDepthImage();

		/* convert to point cloud */
		DepthImageToPointCloudTransformation *img2cloudTramsformer = new DepthImageToPointCloudTransformation();
		img2cloudTramsformer->transformDepthImageToPointCloud(depthImage, pointCloud1, 0);

		delete img2cloudTramsformer;
		delete depthImgageLoader;

	} else {
		pointCloud1->readFromTxtFile(filename1);
		if(nInputClouds == 2) { // load second point cloud
			pointCloud2->readFromTxtFile(filename2);
		}
	}
	cout << "Size of point cloud: " << pointCloud1->getSize() << endl;


	/* reduce point cloud with Octree filter */
	Octree* octreeFilter = new Octree();
	octreeFilter->setVoxelSize(4.0); 	//value deduced from roughly knowing the data in advance...
	PointCloud3D* reducedPointCloud = new PointCloud3D();
	octreeFilter->filter(pointCloud1, reducedPointCloud);

	/* optionally perform registration via ICP */
	if(nInputClouds == 2) {
		IterativeClosestPointFactory* icpFactory = new IterativeClosestPointFactory();
		IIterativeClosestPointPtr icp;
		icp = icpFactory->createIterativeClosestPoint(); //take default ICP, otherwise pass a configuration file as parameter
		IHomogeneousMatrix44* resultTransformation = new HomogeneousMatrix44();
		icp->match(pointCloud1, pointCloud2, resultTransformation);

		/* here we use the streaming functionality to append a point cloud to another */
		stringstream tmpSteam;
		tmpSteam << *pointCloud2;
		tmpSteam >> *pointCloud1;
	}

	cout << "Size of point cloud: " << pointCloud1->getSize() << endl;

	/* visualize the point cloud */
	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
	visualizer->visualizePointCloud(pointCloud1);

	/* create mesh */
	ITriangleMesh* mesh = new TriangleMeshExplicit();
	DelaunayTriangulationOSG* meshGenerator = new DelaunayTriangulationOSG();
	meshGenerator->triangulate(pointCloud1, mesh);
	cout << "Number of generated triangles: " << mesh->getSize() << endl;

	/* visualize mesh */
	OSGTriangleMeshVisualizer* meshVisualizer = new OSGTriangleMeshVisualizer();
	meshVisualizer->addTriangleMesh(mesh);
	meshVisualizer->visualize();

	/* clean up */
	delete meshVisualizer;
	delete meshGenerator;
	delete visualizer;
	delete pointCloud1;
	delete pointCloud2;

	return 0;
}

/* EOF */
