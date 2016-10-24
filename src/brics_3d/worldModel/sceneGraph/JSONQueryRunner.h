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
#include "brics_3d/worldModel/sceneGraph/JSONDeserializer.h"

namespace brics_3d {
namespace rsg {

/**
 * @brief Accepts a JSON based queries and returns a JSON based result.
 */
class JSONQueryRunner {
public:

	/**
	 * @brief Constructor
	 * @param wm Handle to perform the queries on.
	 */
	JSONQueryRunner(WorldModel* wm);

	/**
	 * @brief Constructor to be used with intermediate filters for UPDATE queries.
	 * @param wm The WorldModel for querying.
	 * @param sceneUpdater The scene to be updated (only foe UPDATE queries). Typically an update filter.
	 */
	JSONQueryRunner(WorldModel* wm, ISceneGraphUpdate* sceneUpdater);

	/**
	 * @brief Default destructor
	 */
	virtual ~JSONQueryRunner();

	/**
	 * @brief Processes a JSON based queries and returns a JSON based result.
	 * @param queryAsJson Query that compiles to the rsg-query-schema.json or rsg-update-schema.json Schema.
	 * @param resultAsJson Result of a query that complies to the rsg-queryResult-schema.json Schema or an error object.
	 *                     The error object is inspired by the Google JSON guidelines.
	 * @return True on success full execution (No coding errors, etc). Though, the result could be empty.
	 */
	bool query(std::string& queryAsJson, std::string& resultAsJson);

private:

	bool query(libvariant::Variant& query, libvariant::Variant& result);

	WorldModel* wm;

	ISceneGraphUpdate* sceneUpdater;

	/// Responsible for update queries. In particular useful to get feedback if a node has been created.
	JSONDeserializer* updateOperationRunner;

	/* Query related handlers */
	bool handleGetNodes(libvariant::Variant& query, libvariant::Variant& result);
	bool handleGetNodeAttributes(libvariant::Variant& query, libvariant::Variant& result);
	bool handleGetNodeParents(libvariant::Variant& query, libvariant::Variant& result);
	bool handleGetGroupChildren(libvariant::Variant& query, libvariant::Variant& result);
	bool handleGetRootNode(libvariant::Variant& query, libvariant::Variant& result);
	bool handleGetRemoteRootNodes(libvariant::Variant& query, libvariant::Variant& result);
	bool handleGetTransform(libvariant::Variant& query, libvariant::Variant& result);
	bool handleGetGeometry(libvariant::Variant& query, libvariant::Variant& result);
	bool handleGetSourceIds(libvariant::Variant& query, libvariant::Variant& result);
	bool handleGetTargetIds(libvariant::Variant& query, libvariant::Variant& result);

	/**
	 * @brief Sets up an error message as as result.
	 * @param[in] message Message text to be written. Currently there are no error codes.
	 * @param[out] result Reference to the result message. The data will be _replaced: by the error message.
	 */
	void handleError(std::string message, libvariant::Variant& result);

	/* FunctionBlock (query) related handlers */
	bool handleLoadFunctionBlock(libvariant::Variant& query, libvariant::Variant& result);
	bool handleUnloadFunctionBlock(libvariant::Variant& query, libvariant::Variant& result);
	bool handleExecuteFunctionBlock(libvariant::Variant& query, libvariant::Variant& result);
	bool handleConfigureFunctionBlock(libvariant::Variant& query, libvariant::Variant& result);
	bool handleGetMetaModelOfFunctionBlock(libvariant::Variant& query, libvariant::Variant& result);

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_JSONQUERYRUNNER_H_ */

/* EOF */
