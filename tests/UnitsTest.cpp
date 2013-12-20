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

#include "UnitsTest.h"
#include <math.h>

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( UnitsTest );

void UnitsTest::setUp() {
	Units unit;
}

void UnitsTest::tearDown() {

}

void UnitsTest::testTimeUnits() {
//	Units::TimeUnit unit;
//	unit = Units::Second;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, 1e9 * 1e-9, maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::timeToSeconds(1.0, Units::Second), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-9, Units::timeToSeconds(1.0, Units::NanoSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-6, Units::timeToSeconds(1.0, Units::MicroSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, Units::timeToSeconds(1.0, Units::MilliSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(60.0, Units::timeToSeconds(1.0, Units::Minute), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3600.0, Units::timeToSeconds(1.0, Units::Hour), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(86400.0, Units::timeToSeconds(1.0, Units::Day), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::timeToSeconds(1e9, Units::NanoSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::timeToSeconds(1e6, Units::MicroSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::timeToSeconds(1e3, Units::MilliSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::timeToSeconds(1.0/60.0, Units::Minute), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::timeToSeconds(1.0/3600.0, Units::Hour), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::timeToSeconds(1.0/86400.0, Units::Day), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e9, Units::getScale(Units::Second, Units::NanoSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-9, Units::getScale(Units::NanoSecond, Units::Second), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::getScale(Units::MicroSecond, Units::NanoSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(24, Units::getScale(Units::Day, Units::Hour), maxTolerance);
}

void UnitsTest::testDistanceUnits() {

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-9, Units::distanceToMeters(1.0, Units::NanoMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-6, Units::distanceToMeters(1.0, Units::MicroMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, Units::distanceToMeters(1.0, Units::MilliMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-2, Units::distanceToMeters(1.0, Units::CentiMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-1, Units::distanceToMeters(1.0, Units::DeciMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::distanceToMeters(1.0, Units::Meter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::distanceToMeters(1.0, Units::KiloMeter), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::distanceToMeters(1e9, Units::NanoMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::distanceToMeters(1e6, Units::MicroMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::distanceToMeters(1e3, Units::MilliMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::distanceToMeters(1e2, Units::CentiMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::distanceToMeters(1e1, Units::DeciMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::distanceToMeters(1.0, Units::Meter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::distanceToMeters(1e-3, Units::KiloMeter), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e9, Units::getScale(Units::Meter, Units::NanoMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-9, Units::getScale(Units::NanoMeter, Units::Meter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, Units::getScale(Units::NanoMeter, Units::NanoMeter), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::getScale(Units::KiloMeter, Units::Meter), maxTolerance);

}

void UnitsTest::testVelocityUnits() {
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, Units::velocityToMetersPerSecond(1.0, Units::MeterPerSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.6, Units::velocityToMetersPerSecond(1.0, Units::KiloMeterPerHour), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, Units::velocityToMetersPerSecond(1.0, Units::MeterPerSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, Units::velocityToMetersPerSecond(1.0/3.6, Units::KiloMeterPerHour), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1/3.6, Units::getScale(Units::MeterPerSecond, Units::KiloMeterPerHour), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.6, Units::getScale(Units::KiloMeterPerHour, Units::MeterPerSecond), maxTolerance);

}

void UnitsTest::testAngleUnits() {
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, Units::angleToRadian(1.0, Units::Radian), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(M_PI/180, Units::angleToRadian(1.0, Units::Degree), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1/(M_PI/180), Units::getScale(Units::Radian, Units::Degree), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(M_PI/180, Units::getScale(Units::Degree, Units::Radian), maxTolerance);
}

void UnitsTest::testAngularVelocityUnits() {
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, Units::angularVelocityToRadianPerSecond(1.0, Units::RadianPerSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(M_PI/180, Units::angularVelocityToRadianPerSecond(1.0, Units::DegreePerSecond), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1/(M_PI/180), Units::getScale(Units::RadianPerSecond, Units::DegreePerSecond), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(M_PI/180, Units::getScale(Units::DegreePerSecond, Units::RadianPerSecond), maxTolerance);
}

void UnitsTest::testMassUnits() {
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-6, Units::massToKiloGrams(1.0, Units::MilliGram), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, Units::massToKiloGrams(1.0, Units::Gram), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, Units::massToKiloGrams(1.0, Units::KiloGram), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::massToKiloGrams(1.0, Units::Ton), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e9, Units::massToKiloGrams(1.0, Units::MegaTon), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::getScale(Units::Gram, Units::MilliGram), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, Units::getScale(Units::MilliGram, Units::Gram), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::getScale(Units::Ton, Units::KiloGram), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e6, Units::getScale(Units::MegaTon, Units::Ton), maxTolerance);

}

void UnitsTest::testTemperatureUnits() {
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0, Units::temperatureToKelvin(0.0, Units::Kelvin), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(273.15, Units::temperatureToKelvin(0.0, Units::Celsius), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(255.927778, Units::temperatureToKelvin(1.0, Units::Farenheit), maxTolerance);
}

void UnitsTest::testFrequencyUnits() {
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, Units::frequencyToHertz(1.0, Units::Hertz), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::frequencyToHertz(1.0, Units::KiloHertz), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e6, Units::frequencyToHertz(1.0, Units::MegaHertz), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e9, Units::frequencyToHertz(1.0, Units::GigaHertz), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::getScale(Units::KiloHertz, Units::Hertz), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, Units::getScale(Units::Hertz, Units::KiloHertz), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::getScale(Units::GigaHertz, Units::MegaHertz), maxTolerance);
}

void UnitsTest::testVoltageUnits() {
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, Units::voltageToVolts(1.0, Units::Volt), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, Units::voltageToVolts(1.0, Units::MilliVolt), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::getScale(Units::Volt, Units::MilliVolt), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, Units::getScale(Units::MilliVolt, Units::Volt), maxTolerance);
}

void UnitsTest::testCurrentUnits() {
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, Units::currentToAmperes(1.0, Units::Ampere), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, Units::currentToAmperes(1.0, Units::MilliAmpere), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e3, Units::getScale(Units::Ampere, Units::MilliAmpere), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1e-3, Units::getScale(Units::MilliAmpere, Units::Ampere), maxTolerance);
}

}

/* EOF */
