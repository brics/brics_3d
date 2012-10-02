/**
 * @file 
 * Point3DDecoratorTest.h
 *
 * @date: Feb 10, 2010
 * @author: sblume
 */

#ifndef POINT3DDECORATORTEST_H_
#define POINT3DDECORATORTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "core/Point3DDecorator.h"
#include "core/ColoredPoint3D.h"
#include "core/Point3DIntensity.h"

#include <Eigen/Geometry>
#include "core/Point3D.h"
#include "core/HomogeneousMatrix44.h"

#include <boost/shared_ptr.hpp>

using namespace std;
using namespace Eigen;
using namespace brics_3d;

namespace unitTests {

class Point3DDecoratorTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( Point3DDecoratorTest );
	CPPUNIT_TEST( testColorDecoration );
	CPPUNIT_TEST( testRecursiveDecoration );
	CPPUNIT_TEST( testRecursiveDecorationCopies );
	CPPUNIT_TEST( testAddition );
	CPPUNIT_TEST( testSubtraction );
	CPPUNIT_TEST( testMultiplication );
	CPPUNIT_TEST( testTransfomration );
	CPPUNIT_TEST( testStreaming );
	CPPUNIT_TEST( testRawAccess );
	CPPUNIT_TEST( testClone );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testColorDecoration();
	void testRecursiveDecoration();
	void testRecursiveDecorationCopies();
	void testAddition();
	void testSubtraction();
	void testMultiplication();
	void testTransfomration();
	void testStreaming();
	void testRawAccess();
	void testClone();


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Required by Eigen2

private:

	/// Test points
	Point3D* point000;
	Point3D* point111; // we plan to use it as a shared reference
	Point3D* pointMinus123;
	Point3D* pointMax;
	Point3D* pointMin;

	ColoredPoint3D* decoratedPointMinus123;

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

#endif /* POINT3DDECORATORTEST_H_ */

/* EOF */
