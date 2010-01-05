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

#include "algorithm/nearestNeighbor/NearestNeighborFLANN.h"
#include "core/HomogeneousMatrix44.h"
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;
using namespace BRICS_3D;

namespace unitTests {

class NearestNeighborTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( NearestNeighborTest );
	CPPUNIT_TEST( testFLANNConstructor );
	CPPUNIT_TEST( testFLANNSimple );
	CPPUNIT_TEST_SUITE_END();


public:
	void setUp();
	void tearDown();

	void testFLANNConstructor();
	void testFLANNSimple();

private:

	INearestNeighbor* abstractNearestNeigbor;
	NearestNeighborFLANN* nearestNeigbor;

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
