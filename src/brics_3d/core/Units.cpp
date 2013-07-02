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

long double Units::timeToSeconds(long double time, TimeUnit unit) {
	return time * timeUnitScales[unit];
}

long double Units::distanceToMeters(long double distance, DistanceUnit unit) {
	return distance * distanceUnitScales[unit];
}

long double Units::velocityToMetersPerSecond(long double velocity, VelocityUnit unit) {
	return velocity * velocityUnitScales[unit];
}

long double Units::angleToDegree(long double angle, AngleUnit unit) {
	return angle * angleUnitScales[unit];
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
	1,		//	MeterPerSecond = 0,
	3.6		//	KiloMeterPerHour = 1
};

double Units::angleUnitScales[2] = {
	1,			//	Radian = 0,
	0.017453292	//	Degree = 1
};

}
/* EOF */
