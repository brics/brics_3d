/**
 * @file 
 * osg_delaunay.cpp
 *
 * @date: Nov 23, 2009
 * @author: sblume
 */

#include <util/DepthImageLoader.h>
#include <util/OSGPointCloudVisualizer.h>
#include <util/OSGTriangleMeshVisualizer.h>
#include <core/PointCloud3D.h>
#include <core/TriangleMeshImplicit.h>
#include <core/TriangleMeshExplicit.h>
#include <algorithm/filter/Octree.h>
#include <algorithm/DepthImageToPointCloudTransformation.h>
#include <algorithm/meshGeneration/DelaunayTriangulationOSG.h>


#include <iostream>
#include <cstring>

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

	/* get depth image */
	IplImage* depthImage;
	DepthImageLoader* depthImgageLoader = new DepthImageLoader();
	depthImage = depthImgageLoader->loadDepthImage(filename);
	depthImgageLoader->displayDepthImage();

	/* convert to point cloud */
	PointCloud3D* pointCloud = new PointCloud3D();
	DepthImageToPointCloudTransformation *img2cloudTramsformer = new DepthImageToPointCloudTransformation();
	img2cloudTramsformer->transformDepthImageToPointCloud(depthImage, pointCloud, 0);
	cout << "Size of point cloud: " << pointCloud->getSize() << endl;

	/* (optionally) reduce  point cloud with octree filter */
	Octree* octreeFilter = new Octree();
	octreeFilter->setVoxelSize(4.0); //value deduce from roughly knowing the bounding box
	PointCloud3D* reducedPointCloud = new PointCloud3D();
	octreeFilter->createOctree(pointCloud, reducedPointCloud);
	delete pointCloud;
	pointCloud = reducedPointCloud;

	/* visualize point cloud */
	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
	visualizer->visualizePointCloud(pointCloud);

	/* create mesh */
//	ITriangleMesh* mesh = new TriangleMeshImplicit();
	ITriangleMesh* mesh = new TriangleMeshExplicit();
	DelaunayTriangulationOSG* meshGenerator = new DelaunayTriangulationOSG();
	meshGenerator->triangulate(pointCloud, mesh);
	cout << "Number of generated triangles: " << mesh->getSize() << endl;
	/* visualize mesh */
	OSGTriangleMeshVisualizer* meshVisualizer = new OSGTriangleMeshVisualizer();
	meshVisualizer->addTriangleMesh(mesh);
	meshVisualizer->visualize();

	delete meshVisualizer;
	delete meshGenerator;
	delete visualizer;
	delete img2cloudTramsformer;
	delete pointCloud;
	delete depthImgageLoader;

	return 0;
}

/* EOF */
