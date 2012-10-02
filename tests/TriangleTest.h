/**
 * @file 
 * TriangleTest.h
 *
 * @date: Feb 24, 2010
 * @author: sblume
 */

#ifndef TRIANGLETEST_H_
#define TRIANGLETEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/core/Triangle.h"
#include "brics_3d/core/HomogeneousMatrix44.h"

namespace unitTests {

using namespace brics_3d;
using namespace Eigen;
using namespace std;

class TriangleTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( TriangleTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testModification );
	CPPUNIT_TEST( testTransformation );
	CPPUNIT_TEST( testStreaming );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testConstructor();
	void testModification();
	void testTransformation();
	void testStreaming();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Required by Eigen2

private:

	/// Test vertices
	Point3D *vertex000;
	Point3D *vertex111;
	Point3D *vertexMinus123;

	/// Test triangles
	Triangle* defaultTriangle;
	Triangle* testTriangle;

	/// Maximum deviation for equality check of double variables
	static const double maxTolerance = 0.00001;

};

}

#endif /* TRIANGLETEST_H_ */

/* EOF */
