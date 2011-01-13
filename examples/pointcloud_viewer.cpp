/**
 * @file 
 * pointcloud_viewer.cpp
 *
 * @date: Dec 16, 2009
 * @author: sblume
 */


#include <iostream>
#include <util/OSGPointCloudVisualizer.h>
#include <core/PointCloud3D.h>
#include <algorithm/filtering/Octree.h>
#include <cstring>

using namespace std;
using namespace BRICS_3D;



int main(int argc, char **argv) {

	/* check argument */
	string filename;
	bool useOctree = false;
	double voxelSize = 0.0;
	if (argc == 1) {
		cout << "Usage: " << argv[0] << " <filename>" << endl;

		char defaultFilename1[255] = { BRICS_MODELS_DIR };
		strcat(defaultFilename1, "/scan1.txt\0");
		filename = defaultFilename1;

		cout << "Trying to get default file: " << filename << endl;
	} else if (argc == 2) {
		filename = argv[1];
		cout << filename << endl;
	} else if (argc == 3) {
		filename = argv[1];
		voxelSize = atof(argv[2]);
		cout << filename << ", with voxel size = " << voxelSize <<endl;
		useOctree = true;
	} else {
		cerr << "Usage: " << argv[0] << " <filename>" << endl;
		return -1;
	}

	/* convert to point cloud */
	PointCloud3D* pointCloud = new PointCloud3D();
	pointCloud->readFromTxtFile(filename);
	cout << "Size of cloud: " << pointCloud->getSize() << endl;

	/* optionally reduce with octree */
	if (useOctree == true) {
		Octree* octreeReductionFilter = new Octree();
		octreeReductionFilter->setVoxelSize(voxelSize);
		PointCloud3D* reducedPointCloud = new PointCloud3D();
		octreeReductionFilter->reducePointCloud(pointCloud, reducedPointCloud);
		delete pointCloud;
		pointCloud = reducedPointCloud;
		cout << "Octree reduction filtering performed with voxel size: " << voxelSize <<endl;
		cout << "Size of reduced cloud: " << pointCloud->getSize() << endl;
	}

	/* visualize point cloud */
	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
	visualizer->visualizePointCloud(pointCloud);
//	visualizer->visualizePointCloud(pointCloud, 0.0f, 1.0f, 0.0f, 1.0f); //green

	return 0;
}

/* EOF */
