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

#include <boost/ptr_container/ptr_vector.hpp>

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


	PointCloud3D* pointCloudCube = new ColoredPointCloud3D();

	Point3D* point000 = new ColoredPoint3D(new Point3D(0,0,0),0.3,1,1);
	Point3D* point001 = new ColoredPoint3D(new Point3D(0,0,1),1,0.4,1);
	Point3D* point011 = new ColoredPoint3D(new Point3D(0,1,1),1,0.3,1);
	Point3D* point010 = new ColoredPoint3D(new Point3D(0,1,0),0.0,1,1);
	Point3D* point100 = new ColoredPoint3D(new Point3D(1,0,0),1,1,0.2);
	Point3D* point101 = new ColoredPoint3D(new Point3D(1,0,1),1,0.3,1);
	Point3D* point111 = new ColoredPoint3D(new Point3D(1,1,1),0.0,1,0.0);
	Point3D* point110 = new ColoredPoint3D(new Point3D(1,1,0),0.2,0.5,1);

	pointCloudCube->addPoint(point000);
	pointCloudCube->addPoint(point001);
	pointCloudCube->addPoint(point011);
//	pointCloudCube->addPoint(point010);
//	pointCloudCube->addPoint(point100);
//	pointCloudCube->addPoint(point101);
//	pointCloudCube->addPoint(point111);
//	pointCloudCube->addPoint(point110);

	std::vector<Point3D*> test;
//	boost::ptr_vector_owner<Point3D> test;
//	std::vector<Point3D> test;
	test.push_back(new ColoredPoint3D(new Point3D(0,1,0),0.0,1,1));

	test.push_back(point101);
	test.push_back(point111);
	test.push_back(point110);

//	test.push_back(new ColoredPoint3D(new Point3D(0,1,0),0.0,1,1));

	ColoredPoint3D* testColoredPoint;
	testColoredPoint = dynamic_cast<ColoredPoint3D*>(point011);
	cout << "testPoint cast1: " << testColoredPoint << endl;
	cout << *testColoredPoint << endl << endl;
	Point3D* testPoint;
	Point3D* testPoint2;
	testPoint = &((*pointCloudCube->getPointCloud())[2]);
	testPoint2 = (test[0]);
//	testPoint2 = &(test[0]);
	cout << *testPoint << endl;
	cout << *testPoint2 << endl;
	testColoredPoint = dynamic_cast<ColoredPoint3D*>(testPoint2);
	cout << "testPoint cast2: " << testColoredPoint << endl;

	test.clear();

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
//	cout << *rgbPointCloud;

	PointCloud3D* pointCloud;
	pointCloud = loader->getPointCloud();
	IHomogeneousMatrix44* matrix = new HomogeneousMatrix44(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0, 0.0);
	pointCloud->homogeneousTransformation(matrix);

	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
//	visualizer->addPointCloud(pointCloud);
//	visualizer->addColoredPointCloud(pointCloud);
//	visualizer->addColoredPointCloud(pointCloudCube);

//	visualizer->visualizeColoredPointCloud(pointCloudCube);
	visualizer->visualizePointCloud(pointCloud);

	delete testColoredPoint;
	delete matrix;
	delete pointCloudCube;
	delete rgbPointCloud;
	delete loader;

}


/* EOF */
