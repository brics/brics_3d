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

namespace brics_3d {

namespace rsg {

/**
 * Simple container for time data.
 * @ingroup sceneGraph
 */
class TimeStamp {
  public:
	TimeStamp(){this->timeStamp = 0.0;};
	TimeStamp(long double timeStamp){this->timeStamp = timeStamp;};
	virtual ~TimeStamp(){};

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

private:
    long double timeStamp;

};

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

