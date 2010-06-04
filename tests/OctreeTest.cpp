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

	CPPUNIT_ASSERT(octreePartition == 0);
	octreePartition = dynamic_cast<IOctreePartition*>(octreeComponent);
	CPPUNIT_ASSERT(octreePartition != 0);

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

	octreeComponent->reducePointCloud(pointCloudCube, pointCloudResult);
	CPPUNIT_ASSERT_EQUAL(10u, pointCloudCube->getSize()); //input size must not change
	CPPUNIT_ASSERT_EQUAL(pointCloudCube->getSize(), pointCloudResult->getSize()); //no filtering involved with standard parameters

	octreeComponent->setVoxelSize(0.05); //no change (too fine grid in this case)
	octreeComponent->reducePointCloud(pointCloudCube, pointCloudResult);
	CPPUNIT_ASSERT_EQUAL(10u, pointCloudResult->getSize());

	octreeComponent->setVoxelSize(0.1);
	octreeComponent->reducePointCloud(pointCloudCube, pointCloudResult);
	CPPUNIT_ASSERT_EQUAL(9u, pointCloudResult->getSize());

	octreeComponent->setVoxelSize(0.2);
	octreeComponent->reducePointCloud(pointCloudCube, pointCloudResult);
	CPPUNIT_ASSERT_EQUAL(8u, pointCloudResult->getSize());

	octreeComponent->setVoxelSize(2.0);
	octreeComponent->reducePointCloud(pointCloudCube, pointCloudResult);
	CPPUNIT_ASSERT_EQUAL(1u, pointCloudResult->getSize());

	/* again no filtering, check if result cloud is independent from input (might be a very subtile error) */
	octreeComponent->setVoxelSize(0.0);
	octreeComponent->reducePointCloud(pointCloudCube, pointCloudResult);
	CPPUNIT_ASSERT_EQUAL(10u, pointCloudResult->getSize());
	Point3D inputPoint;
	Point3D resultPoint;
	inputPoint = (*pointCloudCube->getPointCloud())[6]; //pick one arbitrary point here the 7th in the cloud
	resultPoint = (*pointCloudResult->getPointCloud())[6];
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, inputPoint.getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, inputPoint.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, inputPoint.getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.getZ(), maxTolerance);

	(*pointCloudCube->getPointCloud())[6].setX(2.0); //modify input point cloud
	(*pointCloudCube->getPointCloud())[6].setY(3.0);
	(*pointCloudCube->getPointCloud())[6].setZ(4.0);

	inputPoint = (*pointCloudCube->getPointCloud())[6];
	resultPoint = (*pointCloudResult->getPointCloud())[6];
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, inputPoint.getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, inputPoint.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, inputPoint.getZ(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.getZ(), maxTolerance);

	/* input size must not change after all operations */
	CPPUNIT_ASSERT_EQUAL(10u, pointCloudCube->getSize());



	delete pointCloudResult;
	delete octreeComponent;
}

void OctreeTest::testPartition() {
	octreeComponent = new Octree();
	int pointCount = 0;

	vector<PointCloud3D*>* partition = new vector<PointCloud3D*>();
	CPPUNIT_ASSERT_EQUAL(0, static_cast<int>(partition->size()));

	octreeComponent->partitionPointCloud(pointCloudCube, partition); //only one partition with default parameter 0.0
	CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(partition->size()));

	/*
	 * check if total amount of points is "invariant"
	 * Note that the input point cloud and a merged point cloud (of the partition) might not have the same ordering,
	 * as the algorithm recursively sorts points into buckets. Hence only the size is constant/invariant
	 */
	pointCount = 0;
	for (unsigned int i = 0; i < partition->size(); ++i) {
		for (unsigned int j = 0; j < (*(*partition)[i]).getSize(); ++j) {
			pointCount++;
//			Point3D tmpPoint = (*(*(*partition)[i]).getPointCloud())[j];
//			cout << "Point in partition: " << tmpPoint << endl;
		}
	}
	CPPUNIT_ASSERT_EQUAL(10, pointCount);

	/* little bit of cleaning up */
	for (unsigned int i = 0; i < partition->size(); ++i) {
		if ((*partition)[i] != 0) {
			delete (*partition)[i];
		}
	}

	/* perform test again with another voxel size */
	octreeComponent->setVoxelSize(0.05);
	octreeComponent->partitionPointCloud(pointCloudCube, partition);
	CPPUNIT_ASSERT_EQUAL(10, static_cast<int>(partition->size()));

	/* check if total amount of points is "invariant" */
	pointCount = 0;
	for (unsigned int i = 0; i < partition->size(); ++i) {
		for (unsigned int j = 0; j < (*(*partition)[i]).getSize(); ++j) {
			pointCount++;
//			Point3D tmpPoint = (*(*(*partition)[i]).getPointCloud())[j];
//			cout << "Point in partition: " << tmpPoint << endl;
		}
	}
	CPPUNIT_ASSERT_EQUAL(10, pointCount);

	/* little bit of cleaning up */
	for (unsigned int i = 0; i < partition->size(); ++i) {
		if ((*partition)[i] != 0) {
			delete (*partition)[i];
		}
	}

	/* perform test again with another voxel size */
	octreeComponent->setVoxelSize(0.1);
	octreeComponent->partitionPointCloud(pointCloudCube, partition);
	CPPUNIT_ASSERT_EQUAL(9,static_cast<int>(partition->size()));

	/* check if total amount of points is "invariant" */
	pointCount = 0;
	for (unsigned int i = 0; i < partition->size(); ++i) {
		for (unsigned int j = 0; j < (*(*partition)[i]).getSize(); ++j) {
			pointCount++;
		}
	}
	CPPUNIT_ASSERT_EQUAL(10, pointCount);

	/* little bit of cleaning up */
	for (unsigned int i = 0; i < partition->size(); ++i) {
		if ((*partition)[i] != 0) {
			delete (*partition)[i];
		}
	}

	/* perform test again with another voxel size */
	octreeComponent->setVoxelSize(0.2);
	octreeComponent->partitionPointCloud(pointCloudCube, partition);
	CPPUNIT_ASSERT_EQUAL(8, static_cast<int>(partition->size()));

	/* check if total amount of points is "invariant" */
	pointCount = 0;
	for (unsigned int i = 0; i < partition->size(); ++i) {
		for (unsigned int j = 0; j < (*(*partition)[i]).getSize(); ++j) {
			pointCount++;
		}
	}
	CPPUNIT_ASSERT_EQUAL(10, pointCount);

	/* little bit of cleaning up */
	for (unsigned int i = 0; i < partition->size(); ++i) {
		if ((*partition)[i] != 0) {
			delete (*partition)[i];
		}
	}

	/* perform test again with another voxel size */
	octreeComponent->setVoxelSize(2.0);
	octreeComponent->partitionPointCloud(pointCloudCube, partition);
	CPPUNIT_ASSERT_EQUAL(1, static_cast<int>(partition->size()));

	/* check if total amount of points is "invariant" */
	pointCount = 0;
	for (unsigned int i = 0; i < partition->size(); ++i) {
		for (unsigned int j = 0; j < (*(*partition)[i]).getSize(); ++j) {
			pointCount++;
		}
	}
	CPPUNIT_ASSERT_EQUAL(10, pointCount);

	/* little bit of cleaning up */
	for (unsigned int i = 0; i < partition->size(); ++i) {
		if ((*partition)[i] != 0) {
			delete (*partition)[i];
		}
	}


	delete partition;
	delete octreeComponent;
}

}

/* EOF */
