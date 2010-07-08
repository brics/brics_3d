/**
 * @file 
 * Timer.cpp
 *
 * @date: Feb 5, 2010
 * @author: sblume
 */

#include "Timer.h"

namespace BRICS_3D {

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
