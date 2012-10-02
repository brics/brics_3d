/**
 * @file 
 * OctreeTest.h
 *
 * @date: May 19, 2010
 * @author: sblume
 */

#ifndef OCTREETEST_H_
#define OCTREETEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/algorithm/filtering/IOctreeReductionFilter.h"
#include "brics_3d/algorithm/filtering/IOctreeSetup.h"
#include "brics_3d/algorithm/filtering/IOctreePartition.h"
#include "brics_3d/algorithm/filtering/Octree.h"

using namespace std;
using namespace brics_3d;

namespace unitTests {

class OctreeTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( OctreeTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testSetupInterface );
	CPPUNIT_TEST( testSizeReduction );
	CPPUNIT_TEST( testPartition );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testConstructor();
	void testSetupInterface();
	void testSizeReduction();
	void testPartition();

private:

	Octree* octreeComponent;
	IOctreeSetup* octreeSetup;
	IOctreePartition* octreePartition;

	static const double maxTolerance = 0.00001;

	  PointCloud3D *pointCloudCube;

	  /* points of a simple cube */
	  Point3D *point000;
	  Point3D *point001;
	  Point3D *point011;
	  Point3D *point010;
	  Point3D *point100;
	  Point3D *point101;
	  Point3D *point111;
	  Point3D *point110;
};

}

#endif /* OCTREETEST_H_ */

/* EOF */
