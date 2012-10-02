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
#include <cstdlib>

#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/ColoredPoint3D.h"
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/util/OSGPointCloudVisualizer.h"
#include "brics_3d/util/Timer.h"
#include "brics_3d/util/Benchmark.h"


using namespace std;
using namespace brics_3d;

#define STD_POINT
//#define RGB_POINT

int main(int argc, char **argv) {

	int numberOfRuns = 15/*0*/; //20
	int stepSize = 100000; //500000
	unsigned int seed = 0; // make sure, seed is always the same.
	long double tmpTimeStamp;


	std::srand(seed);
	Timer timer0;
	PointCloud3D* pointCloud = new PointCloud3D();
	PointCloud3D* coloredPointCloud = new PointCloud3D();
	IHomogeneousMatrix44* transfomation = new HomogeneousMatrix44(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 1.0, 2.0, 3.0);

#ifdef STD_POINT
	Benchmark benchHomogenTrans("point3D_cost_homogenTrans");
	benchHomogenTrans.output << "#Size of Point3D datatype: " << sizeof(Point3D) << endl;
	benchHomogenTrans.output <<	"#nPts\t transformTime\t" << endl;
#endif /* STD_POINT */

#ifdef RGB_POINT
	Benchmark benchHomogenTransRGB("point3D_cost_homogenTransRGB");
	benchHomogenTransRGB.output << "#Size of Decorated ColoredPoint3D: " << sizeof(ColoredPoint3D) << endl;
	benchHomogenTransRGB.output <<	"#nPts\t transformTime\t" << endl;
#endif /* RGB_POINT */

	for (int i = 0; i < numberOfRuns; ++i) {

#ifdef STD_POINT
		for (int j = 0; j < stepSize; ++j) {
			Point3D tmpPoint = Point3D(std::rand(), std::rand(), std::rand());
			pointCloud->addPoint(tmpPoint);
		}
		benchHomogenTrans.output << pointCloud->getSize() << "\t";
//		cout << pointCloud->getSize() << "\t";
#endif /* STD_POINT */

#ifdef RGB_POINT
		for (int j = 0; j < stepSize; ++j) {
			Point3D tmpPoint = ColoredPoint3D(new Point3D(std::rand(), std::rand(), std::rand()), 0, 0, 0); //ll ave same color
			coloredPointCloud->addPoint(tmpPoint);
		}
		benchHomogenTransRGB.output << coloredPointCloud->getSize() << "\t";
#endif /* RGB_POINT */


		/*
		 * perform all needed tests
		 */

#ifdef STD_POINT
		/* homogeneousTransformation on std points */
		timer0.reset();
		pointCloud->homogeneousTransformation(transfomation);
		tmpTimeStamp = timer0.getElapsedTime();
		benchHomogenTrans.output << tmpTimeStamp << endl;;
//		cout << timer0.getElapsedTime() << endl;
#endif /* STD_POINT */

#ifdef RGB_POINT
		/* homogeneousTransformation on colored points */
		timer0.reset();
		coloredPointCloud->homogeneousTransformation(transfomation);
		tmpTimeStamp =  timer0.getElapsedTime();
		benchHomogenTransRGB.output << tmpTimeStamp << endl;
//		cout << timer0.getElapsedTime();
#endif /* RGB_POINT */

	}

//	OSGPointCloudVisualizer* visualizer = new OSGPointCloudVisualizer();
//	visualizer->visualizePointCloud(pointCloud);
	delete transfomation;
	delete coloredPointCloud;
	delete pointCloud;
	cout << "Done." << endl;
}


/* EOF */
