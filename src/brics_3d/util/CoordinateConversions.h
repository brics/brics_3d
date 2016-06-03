/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2016, KU Leuven
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

#ifndef RSG_COORDINATECONVERSIONS_H_
#define RSG_COORDINATECONVERSIONS_H_

#include <string>

namespace brics_3d {
namespace rsg {

/**
 * @brief Conversions between UTM and WGS84 GPS coordinates.
 *
 * UTM is a Cartesian space representation. The surface of the is earth is
 * projected onto a 2D plane. This plane is further subdivided into grid tiles.
 * Each tile is identified by a single "zone".
 *
 * Implementation mostly taken from:
 * http://docs.ros.org/hydro/api/mavros/html/gps__conversions_8h_source.html
 */
class CoordinateConversions {
public:

	/**
	 * @brief Helper function to retrieve the UTM zone letter designator.
	 * @param latitude WGS84 latitude
	 * @return UTM Zone
	 */
	static char getUTMLetterDesignator(double latitude);

	/**
	 * @brief Convert WGS84 GPS coordinates into UTM coordinates.
	 * @param[in] latitude WGS84 GPS latitude
	 * @param[in] longitude WGS84 GPS longitude
	 * @param[out] UTMNorthing UTM north
	 * @param[out] UTMEasting UTM east
	 * @param[out] UTMZone Identifier for the UTM grid zone.
	 */
	static void convertLatLontoUTM(const double latitude, const double longitude,
			double& UTMNorthing, double& UTMEasting, std::string& UTMZone);

	/**
	 * @brief Reversed function of convertLatLontoUTM().
	 */
	static void convertUTMtoLatLon(const double UTMNorthing, const double UTMEasting,
			std::string UTMZone, double& latitude, double& longitude);


};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_COORDINATECONVERSIONS_H_ */

/* EOF */
