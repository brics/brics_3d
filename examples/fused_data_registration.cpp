/**
 * @file 
 * ipa_fused_data_registration.cpp
 *
 * @date: Dec 21, 2009
 * @author: sblume
 */

#include "util/IpaDatasetLoader.h"
#include "util/OSGPointCloudVisualizer.h"
#include "core/HomogeneousMatrix44.h"

#include <iostream>
#include <string>

using namespace BRICS_3D;
using std::cout;
using std::cerr;
using std::endl;
using std::string;

int main(int argc, char **argv) {

	/* check argument */
	string filename;
	if (argc == 1) {
		cout << "Usage: " << argv[0] << " <filename>" << endl;

		char defaultFilename1[255] = { BRICS_IMGAGES_DIR };
		strcat(defaultFilename1, "/pringlesGreenMedian_0\0");
		filename = defaultFilename1;

		cout << "Trying to get default file: " << filename << endl;
	} else if (argc == 2) {
		filename = argv[1];
		cout << filename << endl;
	} else {
		cerr << "Usage: " << argv[0] << " <filename>" << endl;
		return -1;
	}


	IpaDatasetLoader* loader = new IpaDatasetLoader();
	loader->loadColoredPointCloud(filename);

//	PointCloud3D* pointCloud;
//	pointCloud = loader->getPointCloud();
//	cout << "Size of cloud: " << pointCloud->getSize() << endl;
//	cout << *pointCloud;
//
//	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
//	visualizer->visualizePointCloud(pointCloud);

	ColoredPointCloud3D* rgbPointCloud;
	rgbPointCloud = loader->getColoredPointCloud();
	cout << "Size of cloud: " << rgbPointCloud->getSize() << endl;
	cout << *rgbPointCloud;

	PointCloud3D* pointCloud;
	pointCloud = loader->getPointCloud();
	IHomogeneousMatrix44* matrix = new HomogeneousMatrix44(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0, 0.0);
	pointCloud->homogeneousTransformation(matrix);

	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
	visualizer->addPointCloud(pointCloud);
	visualizer->visualizeColoredPointCloud(rgbPointCloud);

	delete rgbPointCloud;
	delete loader;

}


/* EOF */
