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

#ifndef UNCERTAINTYOPERATIONSTEST_H_
#define UNCERTAINTYOPERATIONSTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/core/CovarianceMatrix66.h>


using namespace brics_3d;
//using namespace brics_3d::rsg;
using namespace std;
using std::cout;
using std::endl;


namespace unitTests {

class UncertaintyOperationsTest :  public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( UncertaintyOperationsTest );
	CPPUNIT_TEST( testIdentityCompounding );
	CPPUNIT_TEST( testCompounding );
	CPPUNIT_TEST( testRelativeCompounding );
	CPPUNIT_TEST( testAggregatedCompounding );
	CPPUNIT_TEST( testIdentityMerging );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testIdentityCompounding();
	void testCompounding();
	void testRelativeCompounding();
	void testAggregatedCompounding();
	void testIdentityMerging();


private:

	  /// Maximum deviation for equality check of double variables
	  static const double maxTolerance = 0.00001;

	  /// To properly reset the initial log level
	  brics_3d::Logger::Loglevel initialLogLevel;
};

}

#endif /* UNCERTAINTYOPERATIONSTEST_H_ */

/* EOF */
