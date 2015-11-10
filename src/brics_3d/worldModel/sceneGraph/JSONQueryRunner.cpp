/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2015, KU Leuven
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

#include "JSONQueryRunner.h"

namespace brics_3d {
namespace rsg {

JSONQueryRunner::JSONQueryRunner(WorldModel* wm) :
		wm(wm) {

}

JSONQueryRunner::~JSONQueryRunner() {

}

bool JSONQueryRunner::query(std::string& queryAsJson,
		std::string& resultAsJson) {

	libvariant::Variant query = libvariant:: Deserialize(queryAsJson, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON
	libvariant::Variant result;
	bool success = this->query(query, result);

	resultAsJson = libvariant::Serialize(result, libvariant::SERIALIZE_JSON); //could also go to heap (?!?)
	return success;
}

bool JSONQueryRunner::query(libvariant::Variant& query,
		libvariant::Variant& result) {

	return true;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
