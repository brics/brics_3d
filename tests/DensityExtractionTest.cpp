/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2014, KU Leuven
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

#include "DensityExtractionTest.h"

namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( DensityExtractionTest );

void DensityExtractionTest::setUp() {
	testCloud = new PointCloud3D();
	testCloud->addPoint(Point3D(0,0,0));

	// along x axis
	testCloud->addPoint(Point3D(1,0,0));
	testCloud->addPoint(Point3D(2,0,0));
	testCloud->addPoint(Point3D(-1,0,0));
	testCloud->addPoint(Point3D(-2,0,0));

	// along y axis
	testCloud->addPoint(Point3D(0,1,0));
	testCloud->addPoint(Point3D(0,2,0));
	testCloud->addPoint(Point3D(0,-1,0));
	testCloud->addPoint(Point3D(0,-2,0));
	testCloud->addPoint(Point3D(0,-3,0));

	// along z axis
	testCloud->addPoint(Point3D(0,0,1));
	testCloud->addPoint(Point3D(0,0,2));
	testCloud->addPoint(Point3D(0,0,3));
	testCloud->addPoint(Point3D(0,0,-1));
	testCloud->addPoint(Point3D(0,0,-2));
	testCloud->addPoint(Point3D(0,0,-2)); //crete some unbalanced distribution -> this should not disturb the bounding box
	testCloud->addPoint(Point3D(0,0,-2));
	testCloud->addPoint(Point3D(0,0,-2));
	testCloud->addPoint(Point3D(0,0,-2));
	testCloud->addPoint(Point3D(0,0,-2));
	testCloud->addPoint(Point3D(0,0,-2));
	testCloud->addPoint(Point3D(0,0,-2));

	testCloudUnitCube = new PointCloud3D();
	testCloudUnitCube->addPoint(Point3D(0,0,0));
	testCloudUnitCube->addPoint(Point3D(0,0,1));
	testCloudUnitCube->addPoint(Point3D(0,1,0));
	testCloudUnitCube->addPoint(Point3D(0,1,1));
	testCloudUnitCube->addPoint(Point3D(1,0,0));
	testCloudUnitCube->addPoint(Point3D(1,0,1));
	testCloudUnitCube->addPoint(Point3D(1,1,0));
	testCloudUnitCube->addPoint(Point3D(1,1,1));

}

void DensityExtractionTest::tearDown() {
	if (testCloud) {
	//	delete testCloud;
		testCloud = 0;
	}
	if (testCloudUnitCube) {
	//	delete testCloudUnitCube;
		testCloudUnitCube = 0;
	}
}

void DensityExtractionTest::testSimpleDensityExtraction() {
	DensityExtractor densityExtractor;

	Density result1 = densityExtractor.computeDensity(testCloudUnitCube);

	CPPUNIT_ASSERT_EQUAL(8, result1.numberOfPoints);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, result1.volume, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0/1.0, result1.density, maxTolerance);

	Density result2 = densityExtractor.computeDensity(testCloud);

	CPPUNIT_ASSERT_EQUAL(22, result2.numberOfPoints);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(100, result2.volume, 1); //the PCA estimation brings in a bit in unaccuracy => higher tolerance
	CPPUNIT_ASSERT_DOUBLES_EQUAL(22.0/100.0, result2.density, 0.001);


}

} /* namespace unitTests */



/* EOF */
