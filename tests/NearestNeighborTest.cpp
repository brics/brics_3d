/**
 * @file 
 * NearestNeighborTest.cpp
 *
 * @date: Jan 5, 2010
 * @author: sblume
 */

#include "NearestNeighborTest.h"

#include <sstream>
#include <stdexcept>

using std::runtime_error;

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
	delete point000;
	delete point001;
	delete point011;
	delete point010;
	delete point100;
	delete point101;
	delete point111;
	delete point110;

	delete pointCloudCube;
	delete pointCloudCubeCopy;
	delete nearestNeigborFLANN;
	delete nearestNeigborANN;
	delete nearestNeigborSTANN;
	delete abstractNearestNeigbor;
}

void NearestNeighborTest::testFLANNConstructor() {
	CPPUNIT_ASSERT(nearestNeigborFLANN == 0);
	nearestNeigborFLANN = new NearestNeighborFLANN();
	CPPUNIT_ASSERT(nearestNeigborFLANN != 0);

	/* polymorph variant */
	CPPUNIT_ASSERT(abstractNearestNeigbor == 0);
	abstractNearestNeigbor = new NearestNeighborFLANN();
	CPPUNIT_ASSERT(abstractNearestNeigbor != 0);
}

void NearestNeighborTest::testFLANNSimple() {
	nearestNeigborFLANN = new NearestNeighborFLANN();
	CPPUNIT_ASSERT_EQUAL(-1, nearestNeigborFLANN->getDimension());
	nearestNeigborFLANN->setData(pointCloudCube);
	CPPUNIT_ASSERT_EQUAL(3, nearestNeigborFLANN->getDimension());

	int indexResult;
	vector<int> resultIndices;

	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

//	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
//		indexResult = nearestNeigbor->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i]);
//		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
//	}
//
//	nearestNeigbor->setMaxDistance(-1);
//	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
//		indexResult = nearestNeigbor->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i]);
//		CPPUNIT_ASSERT_EQUAL(-1, indexResult); // can't find any thing
//	}

}

void NearestNeighborTest::testFLANNExtended() {
	nearestNeigborFLANN = new NearestNeighborFLANN();
	CPPUNIT_ASSERT_EQUAL(-1, nearestNeigborFLANN->getDimension());
	nearestNeigborFLANN->setData(pointCloudCube);
	CPPUNIT_ASSERT_EQUAL(3, nearestNeigborFLANN->getDimension());

	int indexResult;
	vector<int> resultIndices;
	unsigned int k = 1;

	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	nearestNeigborFLANN->setMaxDistance(-1); // nothing should change
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* now increase k */
	for (k = 1; k < pointCloudCube->getSize() ; ++k) {
		for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
			nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices, k);
			CPPUNIT_ASSERT_EQUAL(k, static_cast<unsigned int>(resultIndices.size()));
			indexResult = resultIndices[0];
			CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
		}
	}

	k = pointCloudCube->getSize() + 1; //k is bigger than number of points...
	CPPUNIT_ASSERT_THROW(nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k), runtime_error);

	/* lets see how maxDistance and k influence each other */
	k = pointCloudCube->getSize(); // maximum
	int expectedFoundNeighbors;

	nearestNeigborFLANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(0.999); // slightly before border line to 4 neighbors
	expectedFoundNeighbors = 1;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(sqrt(2.0)-0.001); // slightly before border line to 7 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(sqrt(2.0)); // at border line to 7 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(sqrt(3.0)-0.001); // slightly before border line to 8 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 8;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 8;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	k = 4; // some arbitrary value

	nearestNeigborFLANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 4;
	nearestNeigborFLANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

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
	vector<int> resultIndices;

	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

//	int indexResult;
//
//	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
//		indexResult = nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i]);
//		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
//	}
//
//	nearestNeigborSTANN->setMaxDistance(-1);
//	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
//		indexResult = nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i]);
//		CPPUNIT_ASSERT_EQUAL(-1, indexResult); // can't find any thing
//	}

}

void NearestNeighborTest::testSTANNExtended() {
	nearestNeigborSTANN = new NearestNeighborSTANN();
	CPPUNIT_ASSERT_EQUAL(-1, nearestNeigborSTANN->getDimension());
	nearestNeigborSTANN->setData(pointCloudCube);
	CPPUNIT_ASSERT_EQUAL(3, nearestNeigborSTANN->getDimension());

	int indexResult;
	vector<int> resultIndices;
	unsigned int k = 1;

	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	nearestNeigborSTANN->setMaxDistance(-1); // nothing should change
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* now increase k */
	for (k = 1; k < pointCloudCube->getSize() ; ++k) {
		for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
			nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices, k);
			CPPUNIT_ASSERT_EQUAL(k, static_cast<unsigned int>(resultIndices.size()));
			indexResult = resultIndices[0];
			CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
		}
	}

	k = pointCloudCube->getSize() + 1; //k is bigger than number of points...
	CPPUNIT_ASSERT_THROW(nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k), runtime_error);

	/* lets see how maxDistance and k influence each other */
	k = pointCloudCube->getSize(); // maximum
	int expectedFoundNeighbors;

	nearestNeigborSTANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(0.999); // slightly before border line to 4 neighbors
	expectedFoundNeighbors = 1;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(sqrt(2.0)-0.001); // slightly before border line to 7 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(sqrt(2.0)); // at border line to 7 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(sqrt(3.0)-0.001); // slightly before border line to 8 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 8;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 8;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	k = 4; // some arbitrary value

	nearestNeigborSTANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 4;
	nearestNeigborSTANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

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
	vector<int> resultIndices;

	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}


}

void NearestNeighborTest::testANNExtended() {
	nearestNeigborANN = new NearestNeighborANN();
	CPPUNIT_ASSERT_EQUAL(-1, nearestNeigborANN->getDimension());
	nearestNeigborANN->setData(pointCloudCube);
	CPPUNIT_ASSERT_EQUAL(3, nearestNeigborANN->getDimension());

	int indexResult;
	vector<int> resultIndices;
	unsigned int k = 1;

	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	nearestNeigborANN->setMaxDistance(-1); // nothing should change
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* now increase k */
	for (k = 1; k < pointCloudCube->getSize() ; ++k) {
		for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
			nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[i], &resultIndices, k);
			CPPUNIT_ASSERT_EQUAL(k, static_cast<unsigned int>(resultIndices.size()));
			indexResult = resultIndices[0];
			CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
		}
	}

	k = pointCloudCube->getSize() + 1; //k is bigger than number of points...
	CPPUNIT_ASSERT_THROW(nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k), runtime_error);

	/* lets see how maxDistance and k influence each other */
	k = pointCloudCube->getSize(); // maximum
	int expectedFoundNeighbors;

	nearestNeigborANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(0.999); // slightly before border line to 4 neighbors
	expectedFoundNeighbors = 1;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(sqrt(2.0)-0.001); // slightly before border line to 7 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(sqrt(2.0)); // at border line to 7 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(sqrt(3.0)-0.001); // slightly before border line to 8 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 8;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 8;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	k = 4; // some arbitrary value

	nearestNeigborANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 4;
	nearestNeigborANN->findNearestNeigbor(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

}
}

/* EOF */
