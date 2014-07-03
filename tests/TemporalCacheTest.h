/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2013, KU Leuven
*
* Author: Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#ifndef TEMPORALCACHETEST_H_
#define TEMPORALCACHETEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <brics_3d/worldModel/sceneGraph/SceneGraphFacade.h>
#include <brics_3d/worldModel/sceneGraph/TemporalCache.h>


namespace unitTests {

using namespace brics_3d;
//using namespace Eigen;
using namespace std;
using namespace brics_3d::rsg;
using std::cout;
using std::endl;

class TemporalCacheTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( TemporalCacheTest );
	CPPUNIT_TEST( testTimeStamps );
	CPPUNIT_TEST( testSimpleCache );
	CPPUNIT_TEST( testCacheInsertions );
	CPPUNIT_TEST( testCacheDoubleInsertions );
	CPPUNIT_TEST( testCacheConfiguration );
	CPPUNIT_TEST( testGetPrecedingAccessPolicy );
	CPPUNIT_TEST( testGetClosetAccessPolicy );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testTimeStamps();
	void testSimpleCache();
	void testCacheInsertions();
	void testCacheDoubleInsertions();
	void testCacheConfiguration();
	void testGetPrecedingAccessPolicy();
	void testGetClosetAccessPolicy();

private:

	  /// Maximum deviation for equality check of double variables
	  static const double maxTolerance = 0.00001;
};

}

#endif /* TEMPORALCACHETEST_H_ */

/* EOF */
