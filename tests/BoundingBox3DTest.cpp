/**
 * @file 
 * BoundingBox3DTest.cpp
 *
 * @date: Jun 6, 2012
 * @author: sblume
 */

#include "BoundingBox3DTest.h"

namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( BoundingBox3DTest );

void BoundingBox3DTest::setUp() {
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

void BoundingBox3DTest::tearDown() {
	if (testCloud) {
		delete testCloud;
		testCloud = 0;
	}
	if (testCloudUnitCube) {
		delete testCloudUnitCube;
		testCloudUnitCube = 0;
	}
}

void BoundingBox3DTest::testSimpleBoundingBox() {

	BoundingBox3DExtractor boundingBoxExtractor;
	BRICS_3D::Point3D resultBoxCenter;
	BRICS_3D::Vector3D resultBoxDimensions;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter.getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter.getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions.getZ(), maxTolerance);

	/* get bounding box */
	boundingBoxExtractor.computeBoundingBox(testCloudUnitCube, resultBoxCenter, resultBoxDimensions);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, resultBoxCenter.getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, resultBoxCenter.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, resultBoxCenter.getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions.getZ(), maxTolerance);


	/* repeat (ther should be no side effects) */
	BRICS_3D::Point3D resultBoxCenter2;
	BRICS_3D::Vector3D resultBoxDimensions2;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter2.getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter2.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter2.getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions2.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions2.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions2.getZ(), maxTolerance);

	/* get bounding box */
	boundingBoxExtractor.computeBoundingBox(testCloudUnitCube, resultBoxCenter2, resultBoxDimensions2);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, resultBoxCenter2.getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, resultBoxCenter2.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, resultBoxCenter2.getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions2.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions2.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions2.getZ(), maxTolerance);


	/* take the other point cloud */
	BRICS_3D::Point3D resultBoxCenter3;
	BRICS_3D::Vector3D resultBoxDimensions3;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter3.getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter3.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter3.getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions3.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions3.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions3.getZ(), maxTolerance);

	boundingBoxExtractor.computeBoundingBox(testCloud, resultBoxCenter3, resultBoxDimensions3);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultBoxCenter3.getX(), maxTolerance); //postconditions
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(-0.5, resultBoxCenter3.getY(), maxTolerance);
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, resultBoxCenter3.getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, resultBoxDimensions3.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, resultBoxDimensions3.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, resultBoxDimensions3.getZ(), maxTolerance);


	/* move the unit box to all negative quadrants */
	Coordinate Xoffset = -10;
	Coordinate Yoffset = -10;
	Coordinate Zoffset = -10;

	HomogeneousMatrix44 transform(1,0,0, 0,1,0, 0,0,1, Xoffset, Yoffset, Zoffset);
	testCloudUnitCube->homogeneousTransformation(&transform);
//	std::cout << *testCloudUnitCube;

	BRICS_3D::Point3D resultBoxCenter4;
	BRICS_3D::Vector3D resultBoxDimensions4;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter4.getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter4.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxCenter4.getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions4.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions4.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions4.getZ(), maxTolerance);

	boundingBoxExtractor.computeBoundingBox(testCloudUnitCube, resultBoxCenter4, resultBoxDimensions4);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5 + Xoffset, resultBoxCenter4.getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5 + Yoffset, resultBoxCenter4.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5 + Zoffset, resultBoxCenter4.getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions4.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions4.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions4.getZ(), maxTolerance);

}

void BoundingBox3DTest::testSimpleOrientedBox() {
	BoundingBox3DExtractor boundingBoxExtractor;
	BRICS_3D::Vector3D resultBoxDimensions;
	HomogeneousMatrix44 resultTransform;

	const double* matrixData;
	matrixData = resultTransform.getRawData();

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixData[12], maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixData[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultBoxDimensions.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultBoxDimensions.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultBoxDimensions.getZ(), maxTolerance);

	/* get bounding box */
	boundingBoxExtractor.computeOrientedBoundingBox(testCloudUnitCube, &resultTransform, resultBoxDimensions);
//	std::cout << "computeOrientedBoundingBox transform " << std::endl << resultTransform << std::endl;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixData[12], maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixData[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixData[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultBoxDimensions.getZ(), maxTolerance);



	/* take the other point cloud */
	BRICS_3D::Vector3D resultBoxDimensions2;
	HomogeneousMatrix44 resultTransform2;
	const double* matrixData2;
	matrixData2 = resultTransform2.getRawData();

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixData2[12], maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixData2[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixData2[14], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions2.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions2.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, resultBoxDimensions2.getZ(), maxTolerance);

	boundingBoxExtractor.computeOrientedBoundingBox(testCloud, &resultTransform, resultBoxDimensions2);

//	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixData2[12], maxTolerance); //postconditions
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixData2[13], maxTolerance);
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, matrixData2[14], maxTolerance);
	CPPUNIT_ASSERT(resultBoxDimensions2.getX() < 10);
	CPPUNIT_ASSERT(resultBoxDimensions2.getY() < 10);
	CPPUNIT_ASSERT(resultBoxDimensions2.getZ() < 10);
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, resultBoxDimensions2.getX(), maxTolerance);
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, resultBoxDimensions2.getY(), maxTolerance);
//	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, resultBoxDimensions2.getZ(), maxTolerance);

}

}  // namespace unitTests

/* EOF */
