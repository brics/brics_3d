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

#ifndef BRICS_3D_UNITS_H_
#define BRICS_3D_UNITS_H_


namespace brics_3d {

/**
 * @brief Generic unit of measurements declarations.
 *
 * If not other stated, SI units are used:
 * - [s]
 * - [m]
 * - [m/s]
 * - [rad]
 *
 */
class Units {

public:

	/// Possible units of measurements for time.
	enum TimeUnit {
	    NanoSecond = 0,
	    MicroSecond = 1,
	    MilliSecond = 2,
	    Second = 3,
	    Minute = 4,
	    Hour = 5,
	    Day = 6
	};

	/// Possible units of measurements for distance.
	enum DistanceUnit {
		NanoMeter = 0,
		MicroMeter = 1,
		MilliMeter = 2,
		CentiMeter = 3,
		DeciMeter = 4,
		Meter = 5,
		KiloMeter = 6
	};

	/// Possible units of measurements for velocity.
	enum VelocityUnit {
		MeterPerSecond = 0,
		KiloMeterPerHour = 1
	};

	/// Possible units of measurements for angles.
	enum AngleUnit {
		Radian = 0,
		Degree = 1
	};

	/* ... to be extendend */



	Units();
	virtual ~Units();

	/* Functions to convert from any unit to SI unit */
	static long double timeToSeconds(long double time, TimeUnit unit);
	static long double distanceToMeters(long double distance, DistanceUnit unit);
	static long double velocityToMetersPerSecond(long double velocity, VelocityUnit unit);
	static long double angleToDegree(long double angle, AngleUnit unit);

private:

	/* List of scales in relation to the SI units */
	static double timeUnitScales[7];
	static double distanceUnitScales[7];
	static double velocityUnitScales[2];
	static double angleUnitScales[2];

};


}

#endif /* BRICS_3D_UNITS_H_ */

/* EOF */
