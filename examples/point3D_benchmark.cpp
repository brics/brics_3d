/**
 * @file 
 * point3D_benchmark.cpp
 *
 * @date: Feb 5, 2010
 * @author: sblume
 */

#include <iostream>
#include <cstdlib>

#include "core/PointCloud3D.h"
#include "core/ColoredPoint3D.h"
#include "core/HomogeneousMatrix44.h"
#include "util/OSGPointCloudVisualizer.h"
#include "util/Timer.h"
#include "util/Benchmark.h"


using namespace std;
using namespace BRICS_3D;

int main(int argc, char **argv) {

	int numberOfRuns = 15/*0*/; //20
	int stepSize = 100000; //500000
	unsigned int seed = 0; // make sure, seed is always the same.

	std::srand(seed);
	Timer timer0;
	PointCloud3D* pointCloud = new PointCloud3D();
	PointCloud3D* coloredPointCloud = new PointCloud3D();
	IHomogeneousMatrix44* transfomation = new HomogeneousMatrix44(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 1.0, 2.0, 3.0);
	Benchmark benchHomogenTrans("point3D_cost_homogenTrans");
	Benchmark benchHomogenTransRGB("point3D_cost_homogenTransRGB");

//	cout << "Size of Point3D datatype: " << sizeof(Point3D) << endl;
	benchHomogenTrans.output << "#Size of Point3D datatype: " << sizeof(Point3D) << endl;
//	cout << "#nPts\t transformTime\t" << endl;
	benchHomogenTrans.output <<	"#nPts\t transformTime\t" << endl;

	benchHomogenTransRGB.output << "#Size of Decorated ColoredPoint3D: " << sizeof(ColoredPoint3D) << endl;
	benchHomogenTransRGB.output <<	"#nPts\t transformTime\t" << endl;

	for (int i = 0; i < numberOfRuns; ++i) {

		for (int j = 0; j < stepSize; ++j) {
			Point3D tmpPoint = Point3D(std::rand(), std::rand(), std::rand());
			pointCloud->addPoint(tmpPoint);
		}
		benchHomogenTrans.output << pointCloud->getSize() << "\t";
//		cout << pointCloud->getSize() << "\t";

		for (int j = 0; j < stepSize; ++j) {
			Point3D tmpPoint = ColoredPoint3D(new Point3D(std::rand(), std::rand(), std::rand()), 0, 0, 0); //ll ave same color
			coloredPointCloud->addPoint(tmpPoint);
		}
		benchHomogenTransRGB.output << coloredPointCloud->getSize() << "\t";

		/*
		 * perform all needed tests
		 */

		/* homogeneousTransformation on std points */
		timer0.reset();
		pointCloud->homogeneousTransformation(transfomation);
		benchHomogenTrans.output << timer0.getElapsedTime();
//		cout << timer0.getElapsedTime();

		benchHomogenTrans.output << endl;
//		cout << endl;

		/* homogeneousTransformation on colored points */
		timer0.reset();
		coloredPointCloud->homogeneousTransformation(transfomation);
		benchHomogenTransRGB.output << timer0.getElapsedTime();
//		cout << timer0.getElapsedTime();

		benchHomogenTransRGB.output << endl;
//		cout << endl;

	}

//	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
//	visualizer->visualizePointCloud(pointCloud);
	delete transfomation;
	delete coloredPointCloud;
	delete pointCloud;
	cout << "Done." << endl;
}


/* EOF */
