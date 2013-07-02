/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
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

#ifndef RSG_TIMESTAMP_H
#define RSG_TIMESTAMP_H

#include "brics_3d/core/Units.h"

namespace brics_3d {

namespace rsg {

/**
 * Simple container for time data.
 *
 * Internally SI units are used. That means here: [s].
 *
 * @ingroup sceneGraph
 */
class TimeStamp {
  public:

	/**
	 * @brief Default constuctort that will crate a time stamp with a value of 0.
	 */
	TimeStamp();

	/**
	 * @brief Constructor with a time value.
	 * The constructor is the only possibility to specify a time value. Afterwards the timestamp can only be read.
	 *
	 * @param timeStamp The time stamp vailue
	 * @param unit Accompanying unit of measurement. The default is [s]. Other formats will be internally converted to [s].
	 */
	TimeStamp(long double timeStamp, Units::TimeUnit unit = Units::Second);

	/**
	 * @brief Default constuctor.
	 */
	virtual ~TimeStamp();

	TimeStamp operator-(const TimeStamp &rhs) const;
	TimeStamp operator+(const TimeStamp &rhs) const;
	TimeStamp& operator+=(const TimeStamp &rhs);
	TimeStamp& operator-=(const TimeStamp &rhs);
	bool operator==(const TimeStamp &rhs) const;
	bool operator!=(const TimeStamp &rhs);
	bool operator>(const TimeStamp &rhs) const;
	bool operator<(const TimeStamp &rhs) const;
	bool operator>=(const TimeStamp &rhs) const;
	bool operator<=(const TimeStamp &rhs) const;

	/**
	 * @brief Retrieve the time value in seconds.
	 * @return Time in [s]
	 */
	double getSeconds() const;

private:
	/// Internal representation in [s].
    long double timeStamp;

};

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

