/**
 * @file 
 * Point3DTest.h
 *
 * @date: Oct 16, 2009
 * @author: sblume
 */

#ifndef POINT3DTEST_H_
#define POINT3DTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "core/HomogeneousMatrix44.h"
#include "core/Point3D.h"

using namespace std;
using namespace Eigen;
using namespace brics_3d;

namespace unitTests {

/**
 * @brief UnitTest for the Point3D class
 */
class Point3DTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( Point3DTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testSimleAdditions );
	CPPUNIT_TEST( testComplexAdditions );
	CPPUNIT_TEST( testSimleSubtractions );
	CPPUNIT_TEST( testComplexSubtractions );
	CPPUNIT_TEST( testSimleMultiplications );
	CPPUNIT_TEST( testComplexMultiplications );
	CPPUNIT_TEST( testStreaming );
	CPPUNIT_TEST( testIdentity );
	CPPUNIT_TEST(  testRotation90X	);
	CPPUNIT_TEST(  testRotation90Y	);
	CPPUNIT_TEST(  testRotation90Z	);
	CPPUNIT_TEST(  testTranslation	);
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testConstructor();
	void testSimleAdditions();
	void testComplexAdditions();
	void testSimleSubtractions();
	void testComplexSubtractions();
	void testSimleMultiplications();
	void testComplexMultiplications();
	void testStreaming();
	void testIdentity();
	void testRotation90X();
	void testRotation90Y();
	void testRotation90Z();
	void testTranslation();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Required by Eigen2

private:

	/// Test points
	Point3D *point000;
	Point3D *point111;
	Point3D *pointMinus123;
	Point3D *pointMax;
	Point3D *pointMin;

	/// Maximum deviation for equality check of double variables
	static const double maxTolerance = 0.00001;

	/// Absolute maximum value of "Coordinate"
	Coordinate maxCoordValue;
//	double maxCoordValue;

	/// Absolute minimum value of "Coordinate"
	Coordinate minCoordValue;
//	double minCoordValue;

	/// Eigen2 vector
	Vector3d referenceVector;

	/// Eigen2 container for homogeneous transformations
	Transform3d transformation;
};

}

#endif /* POINT3DTEST_H_ */

/* EOF */
