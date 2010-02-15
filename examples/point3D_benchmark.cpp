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
	IHomogeneousMatrix44* transfomation = new HomogeneousMatrix44(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 1.0, 2.0, 3.0);
	Benchmark benchmark("point3D_cost");

	cout << "Size of Point3D datatype: " << sizeof(Point3D) << endl;
	benchmark.output << "#Size of Point3D datatype: " << sizeof(Point3D) << endl;
	cout << "#nPts\t transformTime\t" << endl;
	benchmark.output <<	"#nPts\t transformTime\t" << endl;

	for (int i = 0; i < numberOfRuns; ++i) {

		for (int j = 0; j < stepSize; ++j) {
			Point3D tmpPoint = Point3D(std::rand(), std::rand(), std::rand());
			pointCloud->addPoint(tmpPoint);
		}
		cout << pointCloud->getSize() << "\t";
		benchmark.output << pointCloud->getSize() << "\t";

		/* perform all needed tests */
		// add

		//subtract

		//multiply
//		double multiplicant = 10.5;


		//transform


		timer0.reset();
		pointCloud->homogeneousTransformation(transfomation);
		cout << timer0.getElapsedTime();
		benchmark.output << timer0.getElapsedTime();

		cout << endl;
		benchmark.output << endl;
	}

//	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
//	visualizer->visualizePointCloud(pointCloud);
	delete transfomation;
	delete pointCloud;
}


/* EOF */
