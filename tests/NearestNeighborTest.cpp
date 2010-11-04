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

	nearestNeigborFLANN = 0;
	nearestNeigborANN = 0;
	nearestNeigborSTANN = 0;
	abstractNearestNeigbor = 0;

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

	if (nearestNeigborFLANN) {
		delete nearestNeigborFLANN;
		nearestNeigborFLANN = 0;
	}

	if (nearestNeigborANN) {
		delete nearestNeigborANN;
		nearestNeigborANN = 0;
	}

	if (nearestNeigborSTANN) {
		delete nearestNeigborSTANN;
		nearestNeigborSTANN = 0;
	}

	if (abstractNearestNeigbor) {
		delete abstractNearestNeigbor;
		abstractNearestNeigbor = 0;
	}

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
		nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

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
		nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	nearestNeigborFLANN->setData(pointCloudCube);
	nearestNeigborFLANN->setMaxDistance(-1); // nothing should change
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* now increase k */
	for (k = 1; k < pointCloudCube->getSize() ; ++k) {
		for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
			nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices, k);
			CPPUNIT_ASSERT_EQUAL(k, static_cast<unsigned int>(resultIndices.size()));
			indexResult = resultIndices[0];
			CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
		}
	}

	k = pointCloudCube->getSize() + 1; //k is bigger than number of points...
	CPPUNIT_ASSERT_THROW(nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k), runtime_error);

	/* lets see how maxDistance and k influence each other */
	k = pointCloudCube->getSize(); // maximum
	int expectedFoundNeighbors;

	nearestNeigborFLANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(0.999); // slightly before border line to 4 neighbors
	expectedFoundNeighbors = 1;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(sqrt(2.0)-0.001); // slightly before border line to 7 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(sqrt(2.0)); // at border line to 7 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(sqrt(3.0)-0.001); // slightly before border line to 8 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 8;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 8;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	k = 4; // some arbitrary value

	nearestNeigborFLANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborFLANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 4;
	nearestNeigborFLANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

}

void NearestNeighborTest::testFLANNHighDimension() {
	nearestNeigborFLANN = new NearestNeighborFLANN();
	CPPUNIT_ASSERT_EQUAL(-1, nearestNeigborFLANN->getDimension());

	int dimension = 128;
	int numberElements = 10;
	double value = 0.0;
	vector< vector<double> > data;

	/* setup test data */
	for (int i = 0; i < numberElements; ++i) {
		vector<double> tmpElement;
		for (int j = 0; j < dimension; ++j) {
			tmpElement.push_back(value);
			value++; // just put increasing value into data
		}
		data.push_back(tmpElement);
	}

	int indexResult;
	vector<int> resultIndices;

	/* find nearest neighbor */
	nearestNeigborFLANN->setData(&data);
	for (unsigned int i = 0;  i < data.size(); ++ i) {
		nearestNeigborFLANN->findNearestNeighbors(&data[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* redo the same */
	nearestNeigborFLANN->setData(&data);
	for (unsigned int i = 0;  i < data.size(); ++ i) {
		nearestNeigborFLANN->findNearestNeighbors(&data[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* now increase k */
	unsigned int k;
	for (k = 1; k < data.size() ; ++k) {
		for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
			nearestNeigborFLANN->findNearestNeighbors(&data[i], &resultIndices, k);
			CPPUNIT_ASSERT(resultIndices.size() == k);
			indexResult = resultIndices[0];
			CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
		}
	}

	/* test error cases */
	int invalidDimension = nearestNeigborFLANN->getDimension() + 1;
	vector<double> invalidQuery(invalidDimension);
	CPPUNIT_ASSERT_THROW(nearestNeigborFLANN->findNearestNeighbors(&invalidQuery, &resultIndices, k), runtime_error);

	k = data.size() + 1; //k is bigger than number of points...
	CPPUNIT_ASSERT_THROW(nearestNeigborFLANN->findNearestNeighbors(&data[0], &resultIndices, k), runtime_error);

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
		nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

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
		nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	nearestNeigborSTANN->setData(pointCloudCube);
	nearestNeigborSTANN->setMaxDistance(-1); // nothing should change
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* now increase k */
	for (k = 1; k < pointCloudCube->getSize() ; ++k) {
		for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
			nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices, k);
			CPPUNIT_ASSERT_EQUAL(k, static_cast<unsigned int>(resultIndices.size()));
			indexResult = resultIndices[0];
			CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
		}
	}

	k = pointCloudCube->getSize() + 1; //k is bigger than number of points...
	CPPUNIT_ASSERT_THROW(nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k), runtime_error);

	/* lets see how maxDistance and k influence each other */
	k = pointCloudCube->getSize(); // maximum
	int expectedFoundNeighbors;

	nearestNeigborSTANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(0.999); // slightly before border line to 4 neighbors
	expectedFoundNeighbors = 1;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(sqrt(2.0)-0.001); // slightly before border line to 7 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(sqrt(2.0)); // at border line to 7 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(sqrt(3.0)-0.001); // slightly before border line to 8 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 8;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 8;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	k = 4; // some arbitrary value

	nearestNeigborSTANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborSTANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 4;
	nearestNeigborSTANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

}

void NearestNeighborTest::testSTANNHighDimension() {
	nearestNeigborSTANN = new NearestNeighborSTANN();
	CPPUNIT_ASSERT_EQUAL(-1, nearestNeigborSTANN->getDimension());

	int dimension = BRICS_3D::STANNDimension;
	int numberElements = 10;
	double value = 0.0;
	vector< vector<double> > data;

	/* setup test data */
	for (int i = 0; i < numberElements; ++i) {
		vector<double> tmpElement;
		for (int j = 0; j < dimension; ++j) {
			tmpElement.push_back(value);
			value++; // just put increasing value into data
		}
		data.push_back(tmpElement);
	}

	int indexResult;
	vector<int> resultIndices;

	/* find nearest neighbor */
	nearestNeigborSTANN->setData(&data);
	for (unsigned int i = 0;  i < data.size(); ++ i) {
		nearestNeigborSTANN->findNearestNeighbors(&data[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* redo the same */
	nearestNeigborSTANN->setData(&data);
	for (unsigned int i = 0;  i < data.size(); ++ i) {
		nearestNeigborSTANN->findNearestNeighbors(&data[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* now increase k */
	unsigned int k;
	for (k = 1; k < data.size() ; ++k) {
		for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
			nearestNeigborSTANN->findNearestNeighbors(&data[i], &resultIndices, k);
			CPPUNIT_ASSERT(resultIndices.size() == k);
			indexResult = resultIndices[0];
			CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
		}
	}

	/* test error cases */
	int invalidDimension = nearestNeigborSTANN->getDimension() + 1;
	vector<double> invalidQuery(invalidDimension);
	CPPUNIT_ASSERT_THROW(nearestNeigborSTANN->findNearestNeighbors(&invalidQuery, &resultIndices, k), runtime_error);

	k = data.size() + 1; //k is bigger than number of points...
	CPPUNIT_ASSERT_THROW(nearestNeigborSTANN->findNearestNeighbors(&data[0], &resultIndices, k), runtime_error);
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
		nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
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
		nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	nearestNeigborANN->setData(pointCloudCube);
	nearestNeigborANN->setMaxDistance(-1); // nothing should change
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices);
		CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(resultIndices.size()));
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* now increase k */
	for (k = 1; k < pointCloudCube->getSize() ; ++k) {
		for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
			nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[i], &resultIndices, k);
			CPPUNIT_ASSERT_EQUAL(k, static_cast<unsigned int>(resultIndices.size()));
			indexResult = resultIndices[0];
			CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
		}
	}

	k = pointCloudCube->getSize() + 1; //k is bigger than number of points...
	CPPUNIT_ASSERT_THROW(nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k), runtime_error);

	/* lets see how maxDistance and k influence each other */
	k = pointCloudCube->getSize(); // maximum
	int expectedFoundNeighbors;

	nearestNeigborANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(0.999); // slightly before border line to 4 neighbors
	expectedFoundNeighbors = 1;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(sqrt(2.0)-0.001); // slightly before border line to 7 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(sqrt(2.0)); // at border line to 7 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(sqrt(3.0)-0.001); // slightly before border line to 8 neighbors
	expectedFoundNeighbors = 7;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 8;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 8;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	k = 4; // some arbitrary value

	nearestNeigborANN->setMaxDistance(0.1); // all neighbors too far away, except the query point
	expectedFoundNeighbors = 1;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(1.0); // at border line to 4 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(sqrt(3.0)); // at  border line to 8 neighbors
	expectedFoundNeighbors = 4;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

	nearestNeigborANN->setMaxDistance(-1); // skip MaxDistance
	expectedFoundNeighbors = 4;
	nearestNeigborANN->findNearestNeighbors(&(*pointCloudCube->getPointCloud())[0], &resultIndices, k);
	CPPUNIT_ASSERT_EQUAL(expectedFoundNeighbors, static_cast<int>(resultIndices.size()));

}

void NearestNeighborTest::testANNHighDimension() {
	nearestNeigborANN = new NearestNeighborANN();
	CPPUNIT_ASSERT_EQUAL(-1, nearestNeigborANN->getDimension());

	int dimension = 128;
	int numberElements = 10;
	double value = 0.0;
	vector< vector<double> > data;

	/* setup test data */
	for (int i = 0; i < numberElements; ++i) {
		vector<double> tmpElement;
		for (int j = 0; j < dimension; ++j) {
			tmpElement.push_back(value);
			value++; // just put increasing value into data
		}
		data.push_back(tmpElement);
	}

	int indexResult;
	vector<int> resultIndices;

	/* find nearest neighbor */
	nearestNeigborANN->setData(&data);
	for (unsigned int i = 0;  i < data.size(); ++ i) {
		nearestNeigborANN->findNearestNeighbors(&data[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* redo the same */
	nearestNeigborANN->setData(&data);
	for (unsigned int i = 0;  i < data.size(); ++ i) {
		nearestNeigborANN->findNearestNeighbors(&data[i], &resultIndices);
		CPPUNIT_ASSERT(resultIndices.size() > 0);
		indexResult = resultIndices[0];
		CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
	}

	/* now increase k */
	unsigned int k;
	for (k = 1; k < data.size() ; ++k) {
		for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
			nearestNeigborANN->findNearestNeighbors(&data[i], &resultIndices, k);
			CPPUNIT_ASSERT(resultIndices.size() == k);
			indexResult = resultIndices[0];
			CPPUNIT_ASSERT_EQUAL(static_cast<int>(i), indexResult); // must find the same (index)
		}
	}

	/* test error cases */
	int invalidDimension = nearestNeigborANN->getDimension() + 1;
	vector<double> invalidQuery(invalidDimension);
	CPPUNIT_ASSERT_THROW(nearestNeigborANN->findNearestNeighbors(&invalidQuery, &resultIndices, k), runtime_error);

	k = data.size() + 1; //k is bigger than number of points...
	CPPUNIT_ASSERT_THROW(nearestNeigborANN->findNearestNeighbors(&data[0], &resultIndices, k), runtime_error);


}

}

/* EOF */
