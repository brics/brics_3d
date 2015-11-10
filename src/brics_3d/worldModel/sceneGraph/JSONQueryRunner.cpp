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

	result.Clear();
	result.Set("success", libvariant::Variant(false));

	try {

		// Start to parse it
		if(query.Contains("@worldmodeltype")) {

			std::string type = query.Get("@worldmodeltype").AsString();
			if(type.compare("RSGQuery") == 0) {
				LOG(DEBUG) << "JSONQueryRunner: Found a model for a Query.";

				/* The actual query */
				if(query.Contains("query"))  {

					std::string queryOperation = query.Get("query").AsString();
					if(queryOperation.compare("GET_NODES") == 0) {
						return handleGetNodes(query, result);

					} else if (queryOperation.compare("GET_NODE_ATTRIBUTES") == 0) {
						return handleGetNodeAttributes(query, result);

					} else if (queryOperation.compare("GET_NODE_PARENTS") == 0) {
						return handleGetNodeParents(query, result);

					} else if (queryOperation.compare("GET_GROUP_CHILDREN") == 0) {
						return handleGetGroupChildren(query, result);

					} else if (queryOperation.compare("GET_ROOT_NODE") == 0) {
						return handleGetRootNode(query, result);

					} else if (queryOperation.compare("GET_REMOTE_ROOT_NODES") == 0) {
						return handleGetRemoteRootNodes(query, result);

					} else if (queryOperation.compare("GET_TRANSFORM") == 0) {
						return handleGetTransform(query, result);

					} else if (queryOperation.compare("GET_GEOMETRY") == 0) {
						return handleGetGeometry(query, result);

					} else {
						LOG(ERROR) << "JSONQueryRunner: Mandatory query fild has unknown value = " << queryOperation;
						return false;
					}

				} else {
					LOG(ERROR) << "JSONQueryRunner: Mandatory query field not set at all.";
					return false;
				}


			} else {
				LOG(ERROR) << "JSONQueryRunner: Mandatory type field not set to RSGQuery. Instead it is = " << type;
				return false;
			}

		} else {
			LOG(WARNING) << "Top level model type @worldmodeltype does not exist.";
		}




		return true;

	} catch (std::exception const & e) {
		LOG(ERROR) << "JSONQueryRunner: Generic parser error: " << e.what() << std::endl << "Omitting this query.";
		return false;
	}

	return false;
}

bool JSONQueryRunner::handleGetNodes(libvariant::Variant& query,
		libvariant::Variant& result) {
	return false;
}

bool JSONQueryRunner::handleGetNodeAttributes(libvariant::Variant& query,
		libvariant::Variant& result) {
	return false;
}

bool JSONQueryRunner::handleGetNodeParents(libvariant::Variant& query,
		libvariant::Variant& result) {
	return false;
}

bool JSONQueryRunner::handleGetGroupChildren(libvariant::Variant& query,
		libvariant::Variant& result) {
	return false;
}

bool JSONQueryRunner::handleGetRootNode(libvariant::Variant& query,
		libvariant::Variant& result) {
	return false;
}

bool JSONQueryRunner::handleGetRemoteRootNodes(libvariant::Variant& query,
		libvariant::Variant& result) {
	return false;
}

bool JSONQueryRunner::handleGetTransform(libvariant::Variant& query,
		libvariant::Variant& result) {
	return false;
}

bool JSONQueryRunner::handleGetGeometry(libvariant::Variant& query,
		libvariant::Variant& result) {
	return false;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
