/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2016, KU Leuven
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

#include "CoordinateConversionTest.h"
#include <boost/regex.hpp>
namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( CoordinateConversionTest );

void CoordinateConversionTest::setUp() {

}

void CoordinateConversionTest::tearDown() {

}

void CoordinateConversionTest::testLatLonToUTM() {

	double lat1 = 50.7756807;
	double lon1 = 7.1849999;
	double lat2 = 50.7757212;
	double lon2 = 7.1850346;

	double utmX1 = 0;
	double utmY1 = 0;
	string zone1 = "";
	double utmX2 = 0;
	double utmY2 = 0;
	string zone2 = "";

	CoordinateConversions::convertLatLontoUTM(lat1, lon1, utmX1, utmY1, zone1);
	CoordinateConversions::convertLatLontoUTM(lat2, lon2, utmX2, utmY2, zone2);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(5626450.6 , utmX1, 0.1);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(372033.3,   utmY1, 0.1);
	CPPUNIT_ASSERT(zone1.compare("32U") == 0);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(5626455, utmX2, 0.1);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(372035.8, utmY2, 0.1);
	CPPUNIT_ASSERT(zone2.compare("32U") == 0);



}

void CoordinateConversionTest::testUTMtoLatLon() {

	double lat1 = 0;
	double lon1 = 0;
	double lat2 = 0;
	double lon2 = 0;

	double utmX1 = 5626450.6;
	double utmY1 = 372033.3;
	string zone1 = "32U";
	double utmX2 = 5626455;
	double utmY2 = 372035.8;
	string zone2 = "32U";

	CoordinateConversions::convertUTMtoLatLon(utmX1, utmY1, zone1, lat1, lon1);
	CoordinateConversions::convertUTMtoLatLon(utmX2, utmY2, zone2, lat2, lon2);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(50.7756807 , lat1, 0.00001);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.1849999,   lon1, 0.00001);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(50.7757212 , lat2, 0.00001);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.1850346,   lon2, 0.00001);

}

} /* namespace unitTests */

/* EOF */
