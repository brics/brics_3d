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

#ifndef RSG_JSONQUERYRUNNER_H_
#define RSG_JSONQUERYRUNNER_H_

#include "brics_3d/util/JSONTypecaster.h"
#include "brics_3d/worldModel/WorldModel.h"

namespace brics_3d {
namespace rsg {

/**
 * @brief Accepts a JSON based queries and returns a JSON based result.
 */
class JSONQueryRunner {
public:

	/**
	 * @brief Constuctor
	 * @param wm Handle to perform the queries on.
	 */
	JSONQueryRunner(WorldModel* wm);

	/**
	 * @brief Default destructor
	 */
	virtual ~JSONQueryRunner();

	/**
	 * @brief Processes a JSON based queries and returns a JSON based result.
	 * @param queryAsJson Query that comples to the rsg-query-schema.json Schema.
	 * @param resultAsJson Result of a query that complies to the rsg-queryResult-schema.json Schema.
	 * @return True on ssuccess full execution (No coding errors, etc). Though, the result could be empty.
	 */
	bool query(std::string& queryAsJson, std::string& resultAsJson);

private:

	bool query(libvariant::Variant& query, libvariant::Variant& result);

	WorldModel* wm;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_JSONQUERYRUNNER_H_ */

/* EOF */
