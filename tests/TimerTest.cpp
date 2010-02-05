/**
 * @file 
 * TimerTest.cpp
 *
 * @date: Feb 5, 2010
 * @author: sblume
 */

#include "TimerTest.h"

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( TimerTest );

void TimerTest::setUp() {

}

void TimerTest::tearDown() {

}

void TimerTest::testSimpleTiming() {
	Timer timer;
	int maxDummyLoop = 100;
	int dummy;

	long double timeStamp0 = timer.getCurrentTime();
	for (int i = 0; i < maxDummyLoop; ++i) {
		int dummy = i + 1;
	}
	CPPUNIT_ASSERT(timer.getCurrentTime() > 0);

	/* check if time is incremented */
	CPPUNIT_ASSERT(timer.getElapsedTime() > 0);
	long double timeStamp1 = timer.getCurrentTime();
	CPPUNIT_ASSERT(timeStamp1 > timeStamp0);

	/* check if reset works */
	long double elapsedTime = timer.getElapsedTime();
	timer.reset();
	CPPUNIT_ASSERT(elapsedTime > timer.getElapsedTime());
}

}

/* EOF */
