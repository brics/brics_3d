/**
 * @file 
 * NearestNeighborTest.cpp
 *
 * @date: Jan 5, 2010
 * @author: sblume
 */

#include "NearestNeighborTest.h"

#include <sstream>

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( NearestNeighborTest );

void NearestNeighborTest::setUp() {
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

	pointCloudCubeCopy = new PointCloud3D();
//	stringstream tmpSteam;
//	tmpSteam << *pointCloudCube;
//	tmpSteam >> *pointCloudCubeCopy;
}

void NearestNeighborTest::tearDown() {
	delete pointCloudCube;
	delete pointCloudCubeCopy;
}

void NearestNeighborTest::testFLANNConstructor() {
	CPPUNIT_ASSERT(nearestNeigbor == 0);
	nearestNeigbor = new NearestNeighborFLANN();
	CPPUNIT_ASSERT(nearestNeigbor != 0);

	/* polymorph variant */
	CPPUNIT_ASSERT(abstractNearestNeigbor == 0);
	abstractNearestNeigbor = new NearestNeighborFLANN();
	CPPUNIT_ASSERT(abstractNearestNeigbor != 0);
}

void NearestNeighborTest::testFLANNSimple() {
	nearestNeigbor = new NearestNeighborFLANN();
	CPPUNIT_ASSERT_EQUAL(-1, nearestNeigbor->getDimension());
	nearestNeigbor->setData(pointCloudCube);
	CPPUNIT_ASSERT_EQUAL(3, nearestNeigbor->getDimension());


	int indexResult;

	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		indexResult = nearestNeigbor->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i]);
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	nearestNeigbor->setMaxDistance(-1);
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		indexResult = nearestNeigbor->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i]);
		CPPUNIT_ASSERT_EQUAL(-1, indexResult); // can't find any thing
	}

}

void NearestNeighborTest::testSTANNConstructor() {
	CPPUNIT_ASSERT(nearestNeigborSTANN == 0);
	nearestNeigborSTANN = new NearestNeighborSTANN();
	CPPUNIT_ASSERT(nearestNeigborSTANN != 0);

	/* polymorph variant */
	CPPUNIT_ASSERT(abstractNearestNeigbor == 0);
	abstractNearestNeigbor = new NearestNeighborSTANN();
	CPPUNIT_ASSERT(abstractNearestNeigbor != 0);
}

void NearestNeighborTest::testSTANNSimple() {
	nearestNeigborSTANN = new NearestNeighborSTANN();
	CPPUNIT_ASSERT_EQUAL(-1, nearestNeigborSTANN->getDimension());
	nearestNeigborSTANN->setData(pointCloudCube);
	CPPUNIT_ASSERT_EQUAL(3, nearestNeigborSTANN->getDimension());


	int indexResult;

	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		indexResult = nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i]);
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	nearestNeigborSTANN->setMaxDistance(-1);
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		indexResult = nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i]);
		CPPUNIT_ASSERT_EQUAL(-1, indexResult); // can't find any thing
	}

}

void NearestNeighborTest::testANNConstructor() {
	CPPUNIT_ASSERT(nearestNeigborANN == 0);
	nearestNeigborANN = new NearestNeighborANN();
	CPPUNIT_ASSERT(nearestNeigborANN != 0);

	/* polymorph variant */
	CPPUNIT_ASSERT(abstractNearestNeigbor == 0);
	abstractNearestNeigbor = new NearestNeighborANN();
	CPPUNIT_ASSERT(abstractNearestNeigbor != 0);
}

void NearestNeighborTest::testANNSimple() {
	nearestNeigborANN = new NearestNeighborANN();
	CPPUNIT_ASSERT_EQUAL(-1, nearestNeigborANN->getDimension());
	nearestNeigborANN->setData(pointCloudCube);
	CPPUNIT_ASSERT_EQUAL(3, nearestNeigborANN->getDimension());


	int indexResult;

	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		indexResult = nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i]);
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	nearestNeigborANN->setMaxDistance(-1);
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		indexResult = nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i]);
		CPPUNIT_ASSERT_EQUAL(-1, indexResult); // can't find any thing
	}

}

}

/* EOF */
