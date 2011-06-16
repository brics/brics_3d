/**
 * @file 
 * PointCloud3DTest.h
 *
 * @date: Oct 22, 2009
 * @author: sblume
 */

#ifndef POINTCLOUD3DTEST_H_
#define POINTCLOUD3DTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "core/PointCloud3D.h"
#include "core/HomogeneousMatrix44.h"

using namespace std;
using namespace BRICS_3D;
using namespace Eigen;

namespace unitTests {

/**
 * @brief UnitTest for the PointCloud3D class
 */
class PointCloud3DTest: public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( PointCloud3DTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testContent );
	//CPPUNIT_TEST( testLimits ); //is time consuming
	CPPUNIT_TEST( testStreaming );
	CPPUNIT_TEST( testTransformation );
	CPPUNIT_TEST_SUITE_END();

public:
	  void setUp();
	  void tearDown();

	  void testConstructor();
	  void testContent();
	  void testLimits();
	  void testStreaming();
	  void testTransformation();

	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Required by Eigen2

private:

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

	  /// Eigen2 container for homogeneous transformations
	  Transform3d transformation;
};

}

#endif /* POINTCLOUD3DTEST_H_ */

/* EOF */
