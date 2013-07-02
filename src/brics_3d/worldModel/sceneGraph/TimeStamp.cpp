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

#include "TimeStamp.h"

namespace brics_3d {

namespace rsg {

TimeStamp::TimeStamp(){
	this->timeStamp = 0.0;
}

TimeStamp::TimeStamp(long double timeStamp, Units::TimeUnit unit){
	if (unit != Units::Second) {
		this->timeStamp = Units::timeToSeconds(timeStamp, unit);
	} else {
		this->timeStamp = timeStamp;
	}
}

TimeStamp::~TimeStamp(){

};

TimeStamp TimeStamp::operator-(const TimeStamp &rhs) const {
	return (TimeStamp(timeStamp - rhs.timeStamp));
}

TimeStamp TimeStamp::operator+(const TimeStamp &rhs) const {
	return ((TimeStamp(timeStamp + rhs.timeStamp)));
}

TimeStamp& TimeStamp::operator+=(const TimeStamp &rhs) {
	*this = *this + rhs;
	return *this;
}

TimeStamp& TimeStamp::operator-=(const TimeStamp &rhs) {
	*this = *this - rhs;
	return *this;
}

bool TimeStamp::operator==(const TimeStamp &rhs) const {
	return timeStamp == rhs.timeStamp;
}

bool TimeStamp::operator!=(const TimeStamp &rhs) {
	return timeStamp != rhs.timeStamp;
}

bool TimeStamp::operator>(const TimeStamp &rhs) const {
	if (timeStamp > rhs.timeStamp) {
		return true;
	}
	return false;
}

bool TimeStamp::operator<(const TimeStamp &rhs) const {
	if (timeStamp < rhs.timeStamp) {
		return true;
	}
	return false;
}

bool TimeStamp::operator>=(const TimeStamp &rhs) const {
	if (timeStamp >= rhs.timeStamp) {
		return true;
	}
	return false;
}

bool TimeStamp::operator<=(const TimeStamp &rhs) const {
	if (timeStamp <= rhs.timeStamp) {
		return true;
	}
	return false;
}

double TimeStamp::getSeconds() const {
	return timeStamp;
}

} // namespace brics_3d::RSG

} // namespace brics_3d

/* EOF */

