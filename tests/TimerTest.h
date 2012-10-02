/**
 * @file 
 * TimerTest.h
 *
 * @date: Feb 5, 2010
 * @author: sblume
 */

#ifndef TIMERTEST_H_
#define TIMERTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/util/Timer.h"

using namespace brics_3d;

namespace unitTests {

class TimerTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( TimerTest );
	CPPUNIT_TEST( testSimpleTiming );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testSimpleTiming();
};

}

#endif /* TIMERTEST_H_ */

/* EOF */
