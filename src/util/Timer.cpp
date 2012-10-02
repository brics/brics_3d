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

#include "Timer.h"

namespace brics_3d {

#ifdef WIN32
static LONG64 winTicksPerSecond = 0;
#endif

Timer::Timer() {
#ifdef WIN32
//	if(!winTicksPerSecond) { //FIXME
//		QueryPerformanceFrequency((LARGE_INTEGER*)&winTicksPerSecond);
//	}
#endif
	reset();
}

Timer::~Timer() {

}

void Timer::reset() {
	startTimeStamp = getCurrentTime();
}
long double Timer::getElapsedTime() {
	currentTimeStamp = getCurrentTime();
	return currentTimeStamp - startTimeStamp;
}
long double Timer::getCurrentTime() {
#ifdef WIN32
//	LARGE_INTEGER tick;
//	QueryPerformanceCounter(&tick);
//	tick.QuadPart = tick.QuadPart * 1.0e3 / winTicksPerSecond;
//	return tick.u.LowPart;
	// TODO implement me
	return -1.0;
#else
	struct timeval thisTime;
	gettimeofday(&thisTime, 0);
	return static_cast<long double>( thisTime.tv_sec * 1.0e3 + thisTime.tv_usec * 1.0e-3);
#endif
}

}

/* EOF */
