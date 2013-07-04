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

#include "Units.h"

namespace brics_3d {

Units::Units() {


}

Units::~Units() {

}

double Units::timeToSeconds(double time, TimeUnit unit) {
	return time * timeUnitScales[unit];
}

double Units::distanceToMeters(double distance, DistanceUnit unit) {
	return distance * distanceUnitScales[unit];
}

double Units::velocityToMetersPerSecond(double velocity, VelocityUnit unit) {
	return velocity * velocityUnitScales[unit];
}

double Units::angleToRadian(double angle, AngleUnit unit) {
	return angle * angleUnitScales[unit];
}

double Units::angularVelocityToRadianPerSecond(double angularVelocity, AngularVelocityUnit unit) {
	return angularVelocity * angularVelocityUnitScales[unit];
}

double Units::massToKiloGrams(double mass, MassUnit unit) {
	return mass * massUnitScales[unit];
}

double Units::temperatureToKelvin(double temperature, TemperatureUnit unit) {
	switch (unit) {
		case Units::Kelvin:
			return temperature;

		case Units::Celsius:
			return temperature + 273.15;

		case Units::Farenheit:
			return (temperature + 459.67) * 5.0/9.0;

		default:
			break;
	}
	return 0;
}

double Units::frequencyToHertz(double frequency, FrequencyUnit unit) {
	return frequency * frequencyUnitScales[unit];
}

double Units::voltageToVolts(double voltage, VoltageUnit unit) {
	return voltage * voltageUnitScales[unit];
}

double Units::currentToAmperes(double current, CurrentUnit unit) {
	return current * currentUnitScales[unit];
}


double Units::getScale(TimeUnit unit, TimeUnit referenceUnit) {
	return timeUnitScales[unit] / timeUnitScales[referenceUnit];
}

double Units::getScale(DistanceUnit unit, DistanceUnit referenceUnit) {
	return distanceUnitScales[unit] / distanceUnitScales[referenceUnit];
}

double Units::getScale(VelocityUnit unit, VelocityUnit referenceUnit) {
	return velocityUnitScales[unit] / velocityUnitScales[referenceUnit];
}

double Units::getScale(AngleUnit unit,  AngleUnit referenceUnit) {
	return angleUnitScales[unit] / angleUnitScales[referenceUnit];
}

double Units::getScale(AngularVelocityUnit unit,  AngularVelocityUnit referenceUnit) {
	return angularVelocityUnitScales[unit] / angularVelocityUnitScales[referenceUnit];
}

double Units::getScale(MassUnit unit,  MassUnit referenceUnit) {
	return massUnitScales[unit] / massUnitScales[referenceUnit];
}

double Units::getScale(FrequencyUnit unit,  FrequencyUnit referenceUnit) {
	return frequencyUnitScales[unit] / frequencyUnitScales[referenceUnit];
}

double Units::getScale(VoltageUnit unit, VoltageUnit referenceUnit) {
	return voltageUnitScales[unit] / voltageUnitScales[referenceUnit];
}

double Units::getScale(CurrentUnit unit, CurrentUnit referenceUnit) {
	return currentUnitScales[unit] / currentUnitScales[referenceUnit];
}

double Units::timeUnitScales[7] = {
	1e-9,	// NanoSecond
	1e-6,	// MicroSecond
	1e-3,	// MilliSecond
	1.0,	// Second
	60,    	// Minute
	3600,   // Hour
	86400,  // Day
};

double Units::distanceUnitScales[7] = {
	1e-9,	// NanoMeter
	1e-6,	// MicroMeter
	1e-3,	// MilliMeter
	1e-2,	// CentiMeter
	1e-1,	// DeciMeter
	1.0,	// Meter
	1e3,	// KiloMeter
};

double Units::velocityUnitScales[2] = {
	1,	//	MeterPerSecond = 0,
	3.6	//	KiloMeterPerHour = 1
};

double Units::angleUnitScales[2] = {
	1,           // Radian = 0,
	0.017453292  // Degree = 1
};

double Units::angularVelocityUnitScales[2] = {
	1,          // RadianPerSecond = 0,
	0.017453292	// DegreePerSecond = 1
};

double Units::massUnitScales[5] = {
	1e-6,   // MilliGram = 0,
	1e-3,   // Gram = 1,
	1,      // KiloGram = 2, <- SI
	1e3,    // Ton = 3,
	1e9,    // MegaTon = 4
};

double Units::frequencyUnitScales[4] = {
	1,      // Hertz = 0,
	1e3,    // KiloHertz = 1,
	1e6,    // MegaHertz = 2,
	1e9,    // GigaHertz = 3
};

double Units::voltageUnitScales[2] = {
	1,      // Volt = 0,
	1e-3    // MilliVolt = 1,
};

double Units::currentUnitScales[2] = {
	1,     // Ampere = 0,
	1e-3   // MilliAmpere = 1
};

}
/* EOF */
