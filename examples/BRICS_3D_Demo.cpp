/**
 * @file 
 * BRICS_3D_Demo.cpp
 *
 * @date: Mar 7, 2010
 * @author: sblume
 */

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


#include <iostream>
#include <cstring>

using namespace std;
using namespace BRICS_3D;



int main(int argc, char **argv) {
	bool loadDepthImage = false; // if true, load depth  image, other wise load from txt file
	int nInputClouds = 1; // number of input point clouds

	/* check argument */
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
		loadDepthImage = false; //TODO implement function that decides automatically by file extension
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


	PointCloud3D* pointCloud1 = new PointCloud3D();
	PointCloud3D* pointCloud2 = new PointCloud3D();
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


	/* (optionally) reduce  point cloud with octree filter */
	Octree* octreeFilter = new Octree();
	octreeFilter->setVoxelSize(4.0); //value deduce from roughly knowing the bounding box
	PointCloud3D* reducedPointCloud = new PointCloud3D();
	octreeFilter->reducePointCloud(pointCloud1, reducedPointCloud);
//	delete pointCloud;
//	pointCloud = reducedPointCloud;


	/* if necessary perform registration  */
	if(nInputClouds == 2) {
		IterativeClosestPointFactory* icpFactory = new IterativeClosestPointFactory();
		IIterativeClosestPointPtr icp;
		icp = icpFactory->createIterativeClosestPoint(); //take default
		IHomogeneousMatrix44* resultTransformation = new HomogeneousMatrix44();
		icp->match(pointCloud1, pointCloud2, resultTransformation);

		stringstream tmpSteam;
		tmpSteam << *pointCloud2;
		tmpSteam >> *pointCloud1;
	}

	cout << "Size of point cloud: " << pointCloud1->getSize() << endl;

	/* visualize point cloud */
	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
	visualizer->visualizePointCloud(pointCloud1);

	/* create mesh */
//	ITriangleMesh* mesh = new TriangleMeshImplicit();
	ITriangleMesh* mesh = new TriangleMeshExplicit();
	DelaunayTriangulationOSG* meshGenerator = new DelaunayTriangulationOSG();
	meshGenerator->triangulate(pointCloud1, mesh);
	cout << "Number of generated triangles: " << mesh->getSize() << endl;
	/* visualize mesh */
	OSGTriangleMeshVisualizer* meshVisualizer = new OSGTriangleMeshVisualizer();
	meshVisualizer->addTriangleMesh(mesh);
	meshVisualizer->visualize();

	delete meshVisualizer;
	delete meshGenerator;
	delete visualizer;
	delete pointCloud1;
	delete pointCloud2;

	return 0;
}

/* EOF */
