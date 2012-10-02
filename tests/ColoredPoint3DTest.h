/**
 * @file 
 * ColoredPoint3DTest.h
 *
 * @date: Dec 21, 2009
 * @author: sblume
 */

#ifndef COLOREDPOINT3DTEST_H_
#define COLOREDPOINT3DTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/ColoredPoint3D.h"

using namespace std;
using namespace Eigen;
using namespace brics_3d;

namespace unitTests {

class ColoredPoint3DTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( ColoredPoint3DTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testStreaming );
	CPPUNIT_TEST( testIdentity );
	CPPUNIT_TEST(  testTranslation	);
	CPPUNIT_TEST_SUITE_END();

public:

	void setUp();
	void tearDown();

	void testConstructor();
	void testStreaming();
	void testIdentity();
	void testTranslation();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Required by Eigen2


private:

	/// Test points

	ColoredPoint3D *point000;
	ColoredPoint3D *point111;
	ColoredPoint3D *pointMinus123;
	ColoredPoint3D *pointMax;
	ColoredPoint3D *pointMin;

	/// Maximum deviation for equality check of double variables
	static const double maxTolerance = 0.00001;

	/// Absolute maximum value of "Coordinate"
	Coordinate maxCoordValue;

	/// Absolute minimum value of "Coordinate"
	Coordinate minCoordValue;

	/// Eigen2 vector
	Vector3d referenceVector;

	/// Eigen2 container for homogeneous transformations
	Transform3d transformation;
};


}

#endif /* COLOREDPOINT3DTEST_H_ */

/* EOF */
