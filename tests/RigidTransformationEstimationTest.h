/**
 * @file 
 * RigidTransformationEstimationTest.h
 *
 * @date: Dec 7, 2009
 * @author: sblume
 */

#ifndef RIGIDTRANSFORMATIONESTIMATIONTEST_H_
#define RIGIDTRANSFORMATIONESTIMATIONTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "core/PointCloud3D.h"
#include "core/HomogeneousMatrix44.h"
#include "core/Logger.h"
#include "algorithm/registration/RigidTransformationEstimationSVD.h"
#include "algorithm/registration/RigidTransformationEstimationQUAT.h"
#include "algorithm/registration/RigidTransformationEstimationHELIX.h"
#include "algorithm/registration/RigidTransformationEstimationAPX.h"
#include "algorithm/registration/RigidTransformationEstimationORTHO.h"

#include <Eigen/Geometry>
#include <iostream>

using namespace std;
using namespace BRICS_3D;

namespace unitTests {





class RigidTransformationEstimationTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( RigidTransformationEstimationTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testSVDTransformation );
	CPPUNIT_TEST( testQUATTransformation );
	CPPUNIT_TEST( testHELIXTransformation );
	CPPUNIT_TEST( testAPXTransformation );
//	CPPUNIT_TEST( testORTHOTransformation ); // TODO throws an exception
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testConstructor();
	void testSVDTransformation();
	void testQUATTransformation();
	void testHELIXTransformation();
	void testAPXTransformation();
	void testORTHOTransformation();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Required by Eigen2

private:

	RigidTransformationEstimationSVD* estimator;
	IRigidTransformationEstimation* abstractEstimator;

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

}  // namespace unitTests

#endif /* RIGIDTRANSFORMATIONESTIMATIONTEST_H_ */

/* EOF */
