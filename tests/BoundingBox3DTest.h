/**
 * @file 
 * BoundingBox3DTest.h
 *
 * @date: Jun 6, 2012
 * @author: sblume
 */

#ifndef BOUNDINGBOX3DTEST_H_
#define BOUNDINGBOX3DTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "algorithm/featureExtraction/BoundingBox3DExtractor.h"
#include "core/HomogeneousMatrix44.h"

using namespace std;
using namespace BRICS_3D;

namespace unitTests {

class BoundingBox3DTest : public CPPUNIT_NS::TestFixture {


	CPPUNIT_TEST_SUITE( BoundingBox3DTest );
	CPPUNIT_TEST( testSimpleBoundingBox );
	CPPUNIT_TEST( testSimpleOrientedBox );
	CPPUNIT_TEST_SUITE_END();
public:

	void setUp();
	void tearDown();

	void testSimpleBoundingBox();
	void testSimpleOrientedBox();

	/// Maximum deviation for equality check of double variables
	static const double maxTolerance = 0.00001;

	PointCloud3D* testCloud;
	PointCloud3D* testCloudUnitCube;

private:

};

}

#endif /* BOUNDINGBOX3DTEST_H_ */

/* EOF */
