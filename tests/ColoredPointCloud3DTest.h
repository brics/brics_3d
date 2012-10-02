/**
 * @file 
 * ColoredPointCloud3DTest.h
 *
 * @date: Dec 22, 2009
 * @author: sblume
 */

#ifndef COLOREDPOINTCLOUD3DTEST_H_
#define COLOREDPOINTCLOUD3DTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/ColoredPoint3D.h"
#include "brics_3d/core/Point3DIntensity.h"

using namespace std;
using namespace brics_3d;
using namespace Eigen;

namespace unitTests {

class ColoredPointCloud3DTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( ColoredPointCloud3DTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testContent );
	//CPPUNIT_TEST( testLimits ); //is time consuming
	CPPUNIT_TEST( testStreaming );
	CPPUNIT_TEST( testTransformation );
	CPPUNIT_TEST( testMassiveData );
	CPPUNIT_TEST( testPolymorphPointCloud );
	CPPUNIT_TEST_SUITE_END();

public:
	  void setUp();
	  void tearDown();

	  void testConstructor();
	  void testContent();
	  void testLimits();
	  void testStreaming();
	  void testTransformation();
	  void testMassiveData();
	  void testPolymorphPointCloud();

	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW //Required by Eigen2

private:

	  PointCloud3D* pointCloud;
	  PointCloud3D* pointCloudCube;
	  PointCloud3D* pointCloudCubeCopy;

	  /* points of a simple cube */
	  ColoredPoint3D* point000;
	  ColoredPoint3D* point001;
	  ColoredPoint3D* point011;
	  ColoredPoint3D* point010;
	  ColoredPoint3D* point100;
	  ColoredPoint3D* point101;
	  ColoredPoint3D* point111;
	  ColoredPoint3D* point110;

	  static const double maxTolerance = 0.00001;

	  /// Eigen2 container for homogeneous transformations
	  Transform3d transformation;
};

}

#endif /* COLOREDPOINTCLOUD3DTEST_H_ */

/* EOF */
