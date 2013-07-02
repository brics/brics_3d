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

#include "TemporalCacheTest.h"

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( TemporalCacheTest );

void TemporalCacheTest::setUp() {

}

void TemporalCacheTest::tearDown() {

}

void TemporalCacheTest::testTimeStamps() {

	TimeStamp t1(0.0);
	CPPUNIT_ASSERT(t1 == t1);
	CPPUNIT_ASSERT(TimeStamp(0.0) == t1);
	CPPUNIT_ASSERT(TimeStamp(0.0) == TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0) > TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0) >= TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(0.0) < TimeStamp(1.0));
	CPPUNIT_ASSERT(TimeStamp(1.0) >= TimeStamp(0.0));

	TimeStamp t2(0.0, Units::Second);
	CPPUNIT_ASSERT(t2 == t1);
	CPPUNIT_ASSERT(TimeStamp(0.0) == t2);
	CPPUNIT_ASSERT(TimeStamp(0.0, Units::Second) == TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) > TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) >= TimeStamp(0.0));
	CPPUNIT_ASSERT(TimeStamp(0.0, Units::Second) < TimeStamp(1.0));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) >= TimeStamp(0.0));

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1.0, Units::Second).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-9, TimeStamp(1.0, Units::NanoSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-6, TimeStamp(1.0, Units::MicroSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, TimeStamp(1.0, Units::MilliSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(60.0, TimeStamp(1.0, Units::Minute).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3600.0, TimeStamp(1.0, Units::Hour).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(86400.0, TimeStamp(1.0, Units::Day).getSeconds(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, 1e9 * 1e-9, maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1e9, Units::NanoSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1e6, Units::MicroSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1e3, Units::MilliSecond).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1.0/60.0, Units::Minute).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1.0/3600.0, Units::Hour).getSeconds(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, TimeStamp(1.0/86400.0, Units::Day).getSeconds(), maxTolerance);

	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) > TimeStamp(1e8, Units::NanoSecond));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) < TimeStamp(1e10, Units::NanoSecond));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) > TimeStamp(1e5, Units::MicroSecond));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) < TimeStamp(1e7, Units::MicroSecond));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) > TimeStamp(1e2, Units::MilliSecond));
	CPPUNIT_ASSERT(TimeStamp(1.0, Units::Second) < TimeStamp(1e4, Units::MilliSecond));
	CPPUNIT_ASSERT(TimeStamp(60.0, Units::Second) > TimeStamp(0.9, Units::Minute));
	CPPUNIT_ASSERT(TimeStamp(60.0, Units::Second) < TimeStamp(1.1, Units::Minute));
	CPPUNIT_ASSERT(TimeStamp(60.0*60.0, Units::Second) > TimeStamp(0.9, Units::Hour));
	CPPUNIT_ASSERT(TimeStamp(60.0*60.0, Units::Second) < TimeStamp(1.1, Units::Hour));
	CPPUNIT_ASSERT(TimeStamp(60.0*60.0*24.0, Units::Second) > TimeStamp(0.9, Units::Day));
	CPPUNIT_ASSERT(TimeStamp(60.0*60.0*24.0, Units::Second) < TimeStamp(1.1, Units::Day));

}

}

/* EOF */
