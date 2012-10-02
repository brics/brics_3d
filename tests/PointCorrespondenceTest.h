/**
 * @file 
 * PointCorrespondenceTest.h
 *
 * @date: Dec 4, 2009
 * @author: sblume
 */

#ifndef POINTCORRESPONDENCETEST_H_
#define POINTCORRESPONDENCETEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "core/PointCloud3D.h"
#include "core/HomogeneousMatrix44.h"
#include "algorithm/registration/PointCorrespondenceKDTree.h"

#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace brics_3d;

namespace unitTests {

class PointCorrespondenceTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( PointCorrespondenceTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testSimpleCorrespondence );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testConstructor();
	void testSimpleCorrespondence();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Required by Eigen2

private:
	IPointCorrespondence* abstractAssigner;
	PointCorrespondenceKDTree* assigner;

	PointCloud3D *pointCloudCube;
	PointCloud3D *pointCloudCubeCopy;

	/* points of a simple cube */
	Point3D *point000;
	Point3D *point001;
	Point3D *point011;
	Point3D *point010;
	Point3D *point100;
	Point3D *point101;
	Point3D *point111;
	Point3D *point110;

	static const double maxTolerance = 0.00001;
};
}

#endif /* POINTCORRESPONDENCETEST_H_ */

/* EOF */
