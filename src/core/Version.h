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

#ifndef VERSION_H_
#define VERSION_H_

#include <string>

namespace brics_3d {

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

/**
 * Helper class to decuce the current verison of BRICS_3D
 */
class Version {
public:

	/**
	 * Default constructor
	 */
	Version();

	/**
	 * Default destructor
	 */
	virtual ~Version();

	/**
	 * Get the current verion of BRICS_3D as human readable string.
	 * @return The version. E.g. "BRICS_3D 0.1.0"
	 */
	static std::string getVersionAsString();

	/**
	 * Returns the version number in a hexadecimal representation.
	 * eg. 0.1.0 -> 0x010,  1.2.0 -> 0x120
	 * @return The version as hexadecimal literal.
	 *
	 * This method could be used to assuere that a certain minimum version of BRICS_3D is used, e.g.
	 * @code
	 * 		assert(brics_3d::Version::getVersion() >= 0x010);  // at least 0.1.0
	 * @endcode
	 */
	static int getVersion();

};

}

#endif /* VERSION_H_ */

/* EOF */
