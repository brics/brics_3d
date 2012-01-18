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


#include <iostream>
#include <util/OSGPointCloudVisualizer.h>
#include <core/PointCloud3D.h>
#include <algorithm/filtering/Octree.h>
#include <cstring>

using namespace std;
using namespace BRICS_3D;


/*
 * This program loads a point cloud from a txt file, optionally filters the data with in Octree based
 * sub-sampling filter to reduce the data and finally visualizes the result.
 */
int main(int argc, char **argv) {

	/* check arguments
	 * In case no arguments are given a default txt file with 3D data will be loaded.
	 * In case one argument is given it will be treated a txt file
	 * In case two arguments are given the first argument specifies the txt file and the second on defines a
	 * voxelsize for an activated Octree based filtering.
	 */
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

	/* load to point cloud */
	PointCloud3D* pointCloud = new PointCloud3D();
	pointCloud->readFromTxtFile(filename);
	cout << "Size of cloud: " << pointCloud->getSize() << endl;

	/* optionally reduce with octree */
	if (useOctree == true) {
		/* create and parameterize the Octree filter */
		Octree* octreeReductionFilter = new Octree();
		octreeReductionFilter->setVoxelSize(voxelSize);

		/* do the filtering */
		PointCloud3D* reducedPointCloud = new PointCloud3D();
		octreeReductionFilter->reducePointCloud(pointCloud, reducedPointCloud);

		delete pointCloud; // here we discard the old data
		pointCloud = reducedPointCloud;
		cout << "Octree reduction filtering performed with voxel size: " << voxelSize <<endl;
		cout << "Size of reduced cloud: " << pointCloud->getSize() << endl;
	}

	/* visualize point cloud */
	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
	visualizer->visualizePointCloud(pointCloud);
//	visualizer->visualizePointCloud(pointCloud, 0.0f, 1.0f, 0.0f, 1.0f); //this will change the color to green

	return 0;
}

/* EOF */
