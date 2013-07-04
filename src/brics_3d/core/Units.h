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
 * @brief Generic declarations and scales for commonly used units of measurement.
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

	enum AngularVelocityUnit {
		RadianPerSecond = 0,
		DegreePerSecond = 1
	};

	enum MassUnit {
		MilliGram = 0,
		Gram = 1,
		KiloGram = 2,
		Ton = 3,
		MegaTon = 4
	};

	enum TemperatureUnit {
		Celsius = 0,
		Kelvin = 1,
		Farenheit = 2
	};

	enum FrequencyUnit {
		Hertz = 0,
		KiloHertz = 1,
		MegaHertz = 2,
		GigaHertz = 3
	};

	enum VoltageUnit {
		Volt = 0,
		MilliVolt = 1,
	};

	enum CurrentUnit {
		Ampere = 0,
		MilliAmpere = 1
	};

	enum ForceUnit {
		Newton = 0
	};


	Units();
	virtual ~Units();

	/* Functions to convert from any unit to the corresponding SI unit */
	static double timeToSeconds(double time, TimeUnit unit);
	static double distanceToMeters(double distance, DistanceUnit unit);
	static double velocityToMetersPerSecond(double velocity, VelocityUnit unit);
	static double angleToRadian(double angle, AngleUnit unit);
	static double angularVelocityToRadianPerSecond(double angularVelocity, AngularVelocityUnit unit);
	static double massToKiloGrams(double mass, MassUnit unit);
	static double temperatureToKelvin(double temperature, TemperatureUnit unit);
	static double frequencyToHertz(double frequency, FrequencyUnit unit);
	static double voltageToVolts(double voltage, VoltageUnit unit);
	static double currentToAmperes(double current, CurrentUnit unit);

	/* unit expressed in scale * reference unit */
	static double getScale(TimeUnit unit, TimeUnit referenceUnit);
	static double getScale(DistanceUnit unit, DistanceUnit referenceUnit);
	static double getScale(VelocityUnit unit, VelocityUnit referenceUnit);
	static double getScale(AngleUnit unit, AngleUnit referenceUnit);
	static double getScale(AngularVelocityUnit unit, AngularVelocityUnit referenceUnit);
	static double getScale(MassUnit unit, MassUnit referenceUnit);
	static double getScale(FrequencyUnit unit, FrequencyUnit referenceUnit);
	static double getScale(VoltageUnit unit, VoltageUnit referenceUnit);
	static double getScale(CurrentUnit unit, CurrentUnit referenceUnit);

private:

	/* List of scales in relation to the SI units */
	static double timeUnitScales[7];
	static double distanceUnitScales[7];
	static double velocityUnitScales[2];
	static double angleUnitScales[2];
	static double angularVelocityUnitScales[2];
	static double massUnitScales[5];
	static double frequencyUnitScales[4];
	static double voltageUnitScales[2];
	static double currentUnitScales[2];

};


}

#endif /* BRICS_3D_UNITS_H_ */

/* EOF */
