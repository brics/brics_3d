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

#ifndef UNITSTEST_H_
#define UNITSTEST_H_


#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <brics_3d/core/Units.h>

using namespace brics_3d;
using namespace std;
using std::cout;
using std::endl;

namespace unitTests {

class UnitsTest :  public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( UnitsTest );
	CPPUNIT_TEST( testTimeUnits );
	CPPUNIT_TEST( testDistanceUnits );
	CPPUNIT_TEST( testVelocityUnits );
	CPPUNIT_TEST( testAngleUnits );
	CPPUNIT_TEST( testAngularVelocityUnits );
	CPPUNIT_TEST( testMassUnits );
	CPPUNIT_TEST( testTemperatureUnits );
	CPPUNIT_TEST( testFrequencyUnits );
	CPPUNIT_TEST( testVoltageUnits );
	CPPUNIT_TEST( testCurrentUnits );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testTimeUnits();
	void testDistanceUnits();
	void testVelocityUnits();
	void testAngleUnits();
	void testAngularVelocityUnits();
	void testMassUnits();
	void testTemperatureUnits();
	void testFrequencyUnits();
	void testVoltageUnits();
	void testCurrentUnits();

private:

	  /// Maximum deviation for equality check of double variables
	  static const double maxTolerance = 0.00001;
};

}

#endif /* UNITSTEST_H_ */

/* EOF */
