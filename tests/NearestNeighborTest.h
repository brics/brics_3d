/**
 * @file 
 * NearestNeighborTest.h
 *
 * @date: Jan 5, 2010
 * @author: sblume
 */

#ifndef NEARESTNEIGHBORTEST_H_
#define NEARESTNEIGHBORTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/algorithm/nearestNeighbor/NearestNeighborFLANN.h"
#include "brics_3d/algorithm/nearestNeighbor/NearestNeighborSTANN.h"
#include "brics_3d/algorithm/nearestNeighbor/NearestNeighborANN.h"
#include "brics_3d/core/HomogeneousMatrix44.h"
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;
using namespace brics_3d;

namespace unitTests {

class NearestNeighborTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( NearestNeighborTest );
	CPPUNIT_TEST( testFLANNConstructor );
	CPPUNIT_TEST( testFLANNSimple );
	CPPUNIT_TEST( testFLANNExtended );
	CPPUNIT_TEST( testFLANNHighDimension );
	CPPUNIT_TEST( testSTANNConstructor );
	CPPUNIT_TEST( testSTANNSimple );
	CPPUNIT_TEST( testSTANNExtended );
	CPPUNIT_TEST( testSTANNHighDimension );
	CPPUNIT_TEST( testANNConstructor );
	CPPUNIT_TEST( testANNSimple );
	CPPUNIT_TEST( testANNExtended );
	CPPUNIT_TEST( testANNHighDimension );
	CPPUNIT_TEST_SUITE_END();


public:
	void setUp();
	void tearDown();

	void testFLANNConstructor();
	void testFLANNSimple();
	void testFLANNExtended();
	void testFLANNHighDimension();
	void testSTANNConstructor();
	void testSTANNSimple();
	void testSTANNExtended();
	void testSTANNHighDimension();
	void testANNConstructor();
	void testANNSimple();
	void testANNExtended();
	void testANNHighDimension();

private:

	INearestNeighbor* abstractNearestNeigbor;
	NearestNeighborFLANN* nearestNeigborFLANN;
	NearestNeighborSTANN* nearestNeigborSTANN;
	NearestNeighborANN* nearestNeigborANN;

	PointCloud3D* pointCloudCube;
	PointCloud3D* pointCloudCubeCopy;

	/* points of a simple cube */
	Point3D* point000;
	Point3D* point001;
	Point3D* point011;
	Point3D* point010;
	Point3D* point100;
	Point3D* point101;
	Point3D* point111;
	Point3D* point110;

	static const double maxTolerance = 0.00001;

};



}

#endif /* NEARESTNEIGHBORTEST_H_ */

/* EOF */
