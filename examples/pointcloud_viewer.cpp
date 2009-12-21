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
#include <cstring>

using namespace std;
using namespace BRICS_3D;



int main(int argc, char **argv) {

	/* check argument */
	string filename;
	if (argc == 1) {
		cout << "Usage: " << argv[0] << " <filename>" << endl;

		char defaultFilename1[255] = { BRICS_MODELS_DIR };
		strcat(defaultFilename1, "/scan1.txt\0");
		filename = defaultFilename1;

		cout << "Trying to get default file: " << filename << endl;
	} else if (argc == 2) {
		filename = argv[1];
		cout << filename << endl;
	} else {
		cerr << "Usage: " << argv[0] << " <filename>" << endl;
		return -1;
	}

	/* convert to point cloud */
	PointCloud3D* pointCloud = new PointCloud3D();
	pointCloud->readFromTxtFile(filename);
	cout << "Size of cloud: " << pointCloud->getSize() << endl;

	/* visualize point cloud */
	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
	visualizer->visualizePointCloud(pointCloud);

	return 0;
}

/* EOF */
