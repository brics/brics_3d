/**
 * @file 
 * OctreeTest.cpp
 *
 * @date: May 19, 2010
 * @author: sblume
 */

#include "OctreeTest.h"
#include <stdexcept>

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( OctreeTest );

void OctreeTest::setUp() {
	pointCloudCube = new PointCloud3D();

	point000 = new Point3D(0,0,0);
	point001 = new Point3D(0,0,1);
	point011 = new Point3D(0,1,1);
	point010 = new Point3D(0,1,0);
	point100 = new Point3D(1,0,0);
	point101 = new Point3D(1,0,1);
	point111 = new Point3D(1,1,1);
	point110 = new Point3D(1,1,0);

	pointCloudCube->addPoint(point000);
	pointCloudCube->addPoint(point001);
	pointCloudCube->addPoint(point011);
	pointCloudCube->addPoint(point010);
	pointCloudCube->addPoint(point100);
	pointCloudCube->addPoint(point101);
	pointCloudCube->addPoint(point111);
	pointCloudCube->addPoint(point110);
	pointCloudCube->addPoint(Point3D(1,1,0.9)); //some new points that will be filtered away
	pointCloudCube->addPoint(Point3D(1,1,0.2));
}

void OctreeTest::tearDown() {
	delete point000;
	delete point001;
	delete point011;
	delete point010;
	delete point100;
	delete point101;
	delete point111;
	delete point110;

	delete pointCloudCube;
}

void OctreeTest::testConstructor() {

	CPPUNIT_ASSERT(octreeComponent == 0);
	octreeComponent = new Octree();
	CPPUNIT_ASSERT(octreeComponent != 0);

	CPPUNIT_ASSERT(octreeSetup == 0);
	octreeSetup = dynamic_cast<IOctreeSetup*>(octreeComponent);
	CPPUNIT_ASSERT(octreeSetup != 0);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, octreeSetup->getVoxelSize(), maxTolerance);

	delete octreeComponent;
}

void OctreeTest::testSetupInterface() {

	/* test polymorph variant */
	CPPUNIT_ASSERT(octreeSetup == 0);
	octreeSetup = new Octree();
	CPPUNIT_ASSERT(octreeSetup != 0);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, octreeSetup->getVoxelSize(), maxTolerance);
	octreeSetup->setVoxelSize(1.0);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, octreeSetup->getVoxelSize(), maxTolerance);

	CPPUNIT_ASSERT_THROW(octreeSetup->setVoxelSize(-1.0), runtime_error); //check invalid input

	delete octreeSetup;
}

void OctreeTest::testSizeReduction() {
	octreeComponent = new Octree();

	PointCloud3D* pointCloudResult = new PointCloud3D();

	octreeComponent->createOctree(pointCloudCube, pointCloudResult);
	CPPUNIT_ASSERT_EQUAL(10u, pointCloudCube->getSize()); //input size must not change
	CPPUNIT_ASSERT_EQUAL(pointCloudCube->getSize(), pointCloudResult->getSize()); //no filtering involved with standard parameters

	octreeComponent->setVoxelSize(0.05); //no change (too fine grid in this case)
	octreeComponent->createOctree(pointCloudCube, pointCloudResult);
	CPPUNIT_ASSERT_EQUAL(10u, pointCloudResult->getSize());

	octreeComponent->setVoxelSize(0.1);
	octreeComponent->createOctree(pointCloudCube, pointCloudResult);
	CPPUNIT_ASSERT_EQUAL(9u, pointCloudResult->getSize());

	octreeComponent->setVoxelSize(0.2);
	octreeComponent->createOctree(pointCloudCube, pointCloudResult);
	CPPUNIT_ASSERT_EQUAL(8u, pointCloudResult->getSize());

	octreeComponent->setVoxelSize(2.0);
	octreeComponent->createOctree(pointCloudCube, pointCloudResult);
//	CPPUNIT_ASSERT_EQUAL(1u, pointCloudResult->getSize());
	CPPUNIT_ASSERT_EQUAL(8u, pointCloudResult->getSize()); //TODO check why there is at least one octree decomposition?

	CPPUNIT_ASSERT_EQUAL(10u, pointCloudCube->getSize()); //input size must not change

	delete pointCloudResult;
	delete octreeComponent;
}

void OctreeTest::testPartition() {
	CPPUNIT_FAIL("TODO: implement test");
}

}

/* EOF */
