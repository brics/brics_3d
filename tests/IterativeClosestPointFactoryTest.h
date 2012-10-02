/**
 * @file 
 * IterativeClosestPointFactoryTest.h
 *
 * @date: Dec 15, 2009
 * @author: sblume
 */

#ifndef ITERATIVECLOSESTPOINTFACTORYTEST_H_
#define ITERATIVECLOSESTPOINTFACTORYTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "algorithm/registration/IterativeClosestPointFactory.h"

using namespace std;
using namespace brics_3d;

namespace unitTests {

class IterativeClosestPointFactoryTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( IterativeClosestPointFactoryTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testDefault );
	CPPUNIT_TEST( testUnitTestConfig );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testConstructor();
	void testDefault();
	void testUnitTestConfig();

private:
	IterativeClosestPointFactory* icpFactory;

	string filename;

	static const double maxTolerance = 0.00001;
};

}

#endif /* ITERATIVECLOSESTPOINTFACTORYTEST_H_ */

/* EOF */
