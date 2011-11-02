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

#ifndef TIMER_H_
#define TIMER_H_

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include <ctime>


namespace BRICS_3D {

/**
 * @brief Timer to measure elapsed time in [ms]
 */
class Timer {
public:

	/**
	 * @brief Standard constructor
	 *
	 * Initializes startTimeStamp by invoking reset().
	 */
	Timer();

	/**
	 * @brief Standard destructor
	 */
	virtual ~Timer();

	/**
	 * @brief Resets the timer
	 *
	 * The startTimeStamp is set to currentTimeStamp, that means the elapsed time is reset to 0
	 */
	void reset();

	/**
	 * @brief Get the elapsed time since last invocation of reset or since construction
	 * @return Returns the elapsed time in [ms.µs]
	 */
	long double getElapsedTime();

	/**
	 * @brief Get the current time
	 * @return Returns the current time in [ms.µs]
	 */
	long double getCurrentTime();

private:
	/// Stores the time stamp of last invocation of reset or construction
	long double startTimeStamp;

	/// Stores the time stamp of last invocation of getCurrentTime
	long double currentTimeStamp;

#ifdef WIN32
	LARGE_INTEGER winTicksPerSecond;
#endif
};

}

#endif /* TIMER_H_ */

/* EOF */
