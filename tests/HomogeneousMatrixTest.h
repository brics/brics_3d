/**
 * @file 
 * HomogeneousMatrixTest.h
 *
 * @date: Nov 26, 2009
 * @author: sblume
 */

#ifndef HOMOGENEOUSMATRIXTEST_H_
#define HOMOGENEOUSMATRIXTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <Eigen/Geometry>
#include "core/HomogeneousMatrix44.h"

namespace unitTests {

using namespace std;
using namespace BRICS_3D;
using namespace Eigen;

/**
 * @brief UnitTest for the IHomogeneousMatrixTest and HomogeneousMatrixTest classes
 */
class HomogeneousMatrixTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( HomogeneousMatrixTest );
	CPPUNIT_TEST( testDefaultConstructor );
	CPPUNIT_TEST( testOveloadedConstructor );
	CPPUNIT_TEST( testEigenConstructor );
	CPPUNIT_TEST( testPolymorphConstructor );
	CPPUNIT_TEST( testStreaming );
	CPPUNIT_TEST( testAssignment );
	CPPUNIT_TEST( testMultiplication );
	CPPUNIT_TEST_SUITE_END();

public:
	  void setUp();
	  void tearDown();

	  void testDefaultConstructor();
	  void testOveloadedConstructor();
	  void testEigenConstructor();
	  void testPolymorphConstructor();
	  void testStreaming();
	  void testAssignment();
	  void testMultiplication();

	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Required by Eigen2

private:

	  /// Homogeneous matrix
	  HomogeneousMatrix44* matrix;

	  /// Maximum deviation for equality check of double variables
	  static const double maxTolerance = 0.00001;

	  /// Pointer to matrix data stores in the HomogeneousMatrix
	  const double* matrixPtr;
};

}

#endif /* HOMOGENEOUSMATRIXTEST_H_ */

/* EOF */
