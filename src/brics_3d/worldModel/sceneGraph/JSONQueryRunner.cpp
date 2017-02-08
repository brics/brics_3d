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
	updateOperationRunner = new JSONDeserializer(wm);
	sceneUpdater = 0;
}

JSONQueryRunner::JSONQueryRunner(WorldModel* wm, ISceneGraphUpdate* sceneUpdater) : wm(wm), sceneUpdater(sceneUpdater) {
	updateOperationRunner = new JSONDeserializer(wm, sceneUpdater); // pass by sceneUpdater to the deserializer. That is enough to enable filtering.
}

JSONQueryRunner::~JSONQueryRunner() {
	if(updateOperationRunner) {
		delete updateOperationRunner;
		updateOperationRunner = 0;
	}
}

bool JSONQueryRunner::query(std::string& queryAsJson,
		std::string& resultAsJson) {

	try {

		libvariant::Variant query = libvariant:: Deserialize(queryAsJson, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON
		libvariant::Variant result;
		bool success = this->query(query, result);

		resultAsJson = libvariant::Serialize(result, libvariant::SERIALIZE_JSON); //could also go to heap (?!?)
		return success;

	} catch (std::exception const & e) {
		LOG(ERROR) << "JSONQueryRunner: Parser error at input query: " << e.what() << std::endl << "Omitting this query.";
		resultAsJson = "{}";
		return false;
	}

}

bool JSONQueryRunner::query(libvariant::Variant& query,
		libvariant::Variant& result) {

	result.Clear();
	result.Set("querySuccess", libvariant::Variant(false)); // defult value; to be overriden
	result.Set("@worldmodeltype", libvariant::Variant("RSGQueryResult"));

	try {

		// Start to parse it
		if(query.Contains("@worldmodeltype")) {

			std::string type = query.Get("@worldmodeltype").AsString();
			if(type.compare("RSGQuery") == 0) {
				LOG(DEBUG) << "JSONQueryRunner: Found a model for a Query.";

				/* Optional queryId */
				if(query.Contains("queryId"))  {
					result.Set("queryId", query.Get("queryId"));
				}

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

					} else if (queryOperation.compare("GET_CONNECTION_SOURCE_IDS") == 0) {
						return handleGetSourceIds(query, result);

					} else if (queryOperation.compare("GET_CONNECTION_TARGET_IDS") == 0) {
						return handleGetTargetIds(query, result);

					} else {
						LOG(ERROR) << "JSONQueryRunner: Mandatory query field has unknown value = " << queryOperation;
						handleError("Syntax error: Mandatory query field has unknown value in RSGQuery", result);
						return false;
					}

				} else {
					handleError("Syntax error: Mandatory query field not set in RSGQuery", result);
					return false;
				}

			} else if (type.compare("RSGUpdate") == 0) {
				LOG(DEBUG) << "JSONQueryRunner: Found a model for an Update.";
				result.Clear();
				result.Set("updateSuccess", libvariant::Variant(false));
				result.Set("@worldmodeltype", libvariant::Variant("RSGUpdateResult"));

				/* Optional queryId */
				if(query.Contains("queryId"))  {
					result.Set("queryId", query.Get("queryId"));
				}

				int error = updateOperationRunner->write(query);
				LOG(DEBUG) <<"JSONQueryRunner: error code for RSGUpdate = " << error;
				if(error > 0) { //NOTE: we dont't have this elaborated error messaging (yet) for RSGUpdate here as compared to the queries.
					result.Set("updateSuccess", libvariant::Variant(true));
					return true;
				} else {
					result.Set("updateSuccess", libvariant::Variant(false));
					return false;
				}

			} else if (type.compare("RSGFunctionBlock") == 0) {
				LOG(DEBUG) << "JSONQueryRunner: Found a model for a FunctionBlock operation.";
				result.Clear();
				result.Set("operationSuccess", libvariant::Variant(false));
				result.Set("@worldmodeltype", libvariant::Variant("RSGFunctionBlockResult"));
				result.Set("metamodel", libvariant::Variant("rsg-functionBlock-schema.json"));

				/* Optional/deprecated operationId (similar to a queryId in case of a query) */
				if(query.Contains("operationId"))  {
					result.Set("operationId", query.Get("operationId"));
				}

				/* Optional queryId TODO not yet in the model,
				 * but since also function blocks can be are used as queries
				 * it makes more sense to be consistent with the field  "queryId"  */
				if(query.Contains("queryId"))  {
					result.Set("queryId", query.Get("queryId"));
				}

				/* The actual operation
			    "functionBlockOperation": {
			      "enum": [
			        "LOAD",
			        "UNLOAD",
			        "EXECUTE",
			        "CONFIGURE",
			        "GET_METAMODEL"
			      ]
			    },
			    */
				if(query.Contains("operation"))  {

					std::string queryOperation = query.Get("operation").AsString();
					if(queryOperation.compare("LOAD") == 0) {
						return handleLoadFunctionBlock(query, result);

					} else if (queryOperation.compare("UNLOAD") == 0) {
						return handleUnloadFunctionBlock(query, result);

					} else if (queryOperation.compare("EXECUTE") == 0) {
						return handleExecuteFunctionBlock(query, result);

					} else if (queryOperation.compare("CONFIGURE") == 0) {
						return handleConfigureFunctionBlock(query, result);

					} else if (queryOperation.compare("GET_METAMODEL") == 0) {
						return handleGetMetaModelOfFunctionBlock(query, result);

					} else {
						LOG(ERROR) << "JSONQueryRunner: Mandatory operation field has unknown value = " << queryOperation;
						handleError("Syntax error: Mandatory operation field has unknown value in RSGFunctionBlock", result);
						return false;
					}

				} else {
					handleError("Syntax error: Mandatory operation field not set in RSGFunctionBlock", result);
					return false;
				}

			} else {
				LOG(ERROR) << "JSONQueryRunner: Syntax error: Mandatory @worldmodeltype field not set in RSGQuery or RSGFunctionBlock. Instead it is = " << type;
				handleError("Syntax error: Mandatory @worldmodeltype field not set in RSGQuery or RSGFunctionBlock", result);
				return false;
			}

		} else {
			handleError("Syntax error: Top level model type @worldmodeltype does not exist.", result);
			return false;
		}


		return true;

	} catch (std::exception const & e) {
		LOG(ERROR) << "JSONQueryRunner: Generic parser error: " << e.what() << std::endl << "Omitting this query.";
		handleError("Syntax error: Generic parser error.", result);
		return false;
	}

	return false;
}

bool JSONQueryRunner::handleGetNodes(libvariant::Variant& query,
		libvariant::Variant& result) {

	/* prepare query */
	std::vector<rsg::Attribute> attributes = JSONTypecaster::getAttributesFromJSON(query);
	std::vector<rsg::Id> ids;
	rsg::Id subgraphId = 0; //NIL

	/* check if subgraphId is set */
	if(query.Contains("subgraphId"))  {
		subgraphId = JSONTypecaster::getIdFromJSON(query, "subgraphId");
	}

	/* perform query */
	bool success = false;
	if(subgraphId.isNil()) { // without subgraphId
		success = wm->scene.getNodes(attributes, ids);
	} else { 				 // with subgraphId
		success = wm->scene.getNodes(attributes, ids, subgraphId);
	}

	/* set up result message */
	result.Set("query", libvariant::Variant("GET_NODES"));
	result.Set("querySuccess", libvariant::Variant(success));
	JSONTypecaster::addIdsToJSON(ids, result, "ids");

	return success;
}

bool JSONQueryRunner::handleGetNodeAttributes(libvariant::Variant& query,
		libvariant::Variant& result) {

	/* prepare query */
	rsg::Id id = JSONTypecaster::getIdFromJSON(query, "id");
	if(id.isNil()) {
		handleError("Syntax error: Wrong or missing id.", result);
		return false;
	}
	std::vector<rsg::Attribute> attributes;
	TimeStamp attributesTimeStamp;

	/* perform query */
	bool success = wm->scene.getNodeAttributes(id, attributes, attributesTimeStamp);

	/* set up result message */
	result.Set("query", libvariant::Variant("GET_NODE_ATTRIBUTES"));
	result.Set("querySuccess", libvariant::Variant(success));
	if(success) {
		JSONTypecaster::addAttributesToJSON(attributes, result);
		JSONTypecaster::addTimeStampToJSON(attributesTimeStamp, result, "timeStamp");
	}


	return success;
}

bool JSONQueryRunner::handleGetNodeParents(libvariant::Variant& query,
		libvariant::Variant& result) {

	/* prepare query */
	rsg::Id id = JSONTypecaster::getIdFromJSON(query, "id");
	if(id.isNil()) {
		handleError("Syntax error: Wrong or missing id.", result);
		return false;
	}
	std::vector<rsg::Id> ids;

	/* perform query */
	bool success = wm->scene.getNodeParents(id, ids);

	/* set up result message */
	result.Set("query", libvariant::Variant("GET_NODE_PARENTS"));
	result.Set("querySuccess", libvariant::Variant(success));
	JSONTypecaster::addIdsToJSON(ids, result, "ids");

	return success;
}

bool JSONQueryRunner::handleGetGroupChildren(libvariant::Variant& query,
		libvariant::Variant& result) {

	/* prepare query */
	rsg::Id id = JSONTypecaster::getIdFromJSON(query, "id");
	if(id.isNil()) {
		handleError("Syntax error: Wrong or missing id.", result);
		return false;
	}
	std::vector<rsg::Id> ids;

	/* perform query */
	bool success = wm->scene.getGroupChildren(id, ids);

	/* set up result message */
	result.Set("query", libvariant::Variant("GET_GROUP_CHILDREN"));
	result.Set("querySuccess", libvariant::Variant(success));
	JSONTypecaster::addIdsToJSON(ids, result, "ids");

	return success;
}

bool JSONQueryRunner::handleGetRootNode(libvariant::Variant& query,
		libvariant::Variant& result) {


	/* perform query */
	bool success = true; // There is always a root node.
	Id rootId = wm->scene.getRootId();

	/* set up result message */
	result.Set("query", libvariant::Variant("GET_ROOT_NODE"));
	result.Set("querySuccess", libvariant::Variant(success));
	JSONTypecaster::addIdToJSON(rootId, result, "rootId");

	return success;
}

bool JSONQueryRunner::handleGetRemoteRootNodes(libvariant::Variant& query,
		libvariant::Variant& result) {

	/* prepare query */
	std::vector<rsg::Id> ids;

	/* perform query */
	bool success = wm->scene.getRemoteRootNodes(ids);

	/* set up result message */
	result.Set("query", libvariant::Variant("GET_REMOTE_ROOT_NODES"));
	result.Set("querySuccess", libvariant::Variant(success));
	JSONTypecaster::addIdsToJSON(ids, result, "ids");

	return success;
}

bool JSONQueryRunner::handleGetTransform(libvariant::Variant& query,
		libvariant::Variant& result) {

	/* prepare query */
	rsg::Id id = JSONTypecaster::getIdFromJSON(query, "id");
	if(id.isNil()) {
		handleError("Syntax error: Wrong or missing id.", result);
		return false;
	}
	rsg::Id idReferenceNode = JSONTypecaster::getIdFromJSON(query, "idReferenceNode");
	rsg::TimeStamp timeStamp = JSONTypecaster::getTimeStampFromJSON(query, "timeStamp");
	if (timeStamp == TimeStamp(-1.0)) {
		LOG(DEBUG) << "JSONQueryRunner::handleGetTransform: Assuming LATEST time stamp.",
		timeStamp = TimeStamp(std::numeric_limits<double>::max()); // We use a stamp that is always newer than any other stamp.
	}
	HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44());

	/* perform query */
	bool success = wm->scene.getTransformForNode(id, idReferenceNode, timeStamp, transform);

	/* set up result message */
	result.Set("query", libvariant::Variant("GET_TRANSFORM"));
	result.Set("querySuccess", libvariant::Variant(success));
	if (success) {
		JSONTypecaster::addTransformToJSON(transform, result, "transform");
	}

	return success;
}

bool JSONQueryRunner::handleGetGeometry(libvariant::Variant& query,
		libvariant::Variant& result) {

	/* prepare query */
	rsg::Id id = JSONTypecaster::getIdFromJSON(query, "id");
	if(id.isNil()) {
		handleError("Syntax error: Wrong or missing id.", result);
		return false;
	}
	Shape::ShapePtr shape;
	TimeStamp timeStamp;

	/* perform query */
	bool success = wm->scene.getGeometry(id, shape, timeStamp);

	/* set up result message */
	result.Set("query", libvariant::Variant("GET_GEOMETRY"));
	result.Set("querySuccess", libvariant::Variant(success));
	if(success) {
		JSONTypecaster::addShapeToJSON(shape, result, "geometry");
		result.Set("unit", libvariant::Variant("m"));
		JSONTypecaster::addTimeStampToJSON(timeStamp, result, "timeStamp");
	}

	return success;
}

bool JSONQueryRunner::handleGetSourceIds(libvariant::Variant& query,
		libvariant::Variant& result) {

	/* prepare query */
	rsg::Id id = JSONTypecaster::getIdFromJSON(query, "id");
	if(id.isNil()) {
		handleError("Syntax error: Wrong or missing id.", result);
		return false;
	}
	std::vector<rsg::Id> ids;

	/* perform query */
	bool success = wm->scene.getConnectionSourceIds(id, ids);

	/* set up result message */
	result.Set("query", libvariant::Variant("GET_CONNECTION_SOURCE_IDS"));
	result.Set("querySuccess", libvariant::Variant(success));
	JSONTypecaster::addIdsToJSON(ids, result, "ids");

	return success;
}

bool JSONQueryRunner::handleGetTargetIds(libvariant::Variant& query,
		libvariant::Variant& result) {

	/* prepare query */
	rsg::Id id = JSONTypecaster::getIdFromJSON(query, "id");
	if(id.isNil()) {
		handleError("Syntax error: Wrong or missing id.", result);
		return false;
	}
	std::vector<rsg::Id> ids;

	/* perform query */
	bool success = wm->scene.getConnectionTargetIds(id, ids);

	/* set up result message */
	result.Set("query", libvariant::Variant("GET_CONNECTION_TARGET_IDS"));
	result.Set("querySuccess", libvariant::Variant(success));
	JSONTypecaster::addIdsToJSON(ids, result, "ids");

	return success;
}

void JSONQueryRunner::handleError(std::string message, libvariant::Variant& result) {
	LOG(ERROR) << "JSONQueryRunner::handleError: " << message;
	result.Clear();
	libvariant::Variant error;
	error.Set("message", libvariant::Variant(message));
	result.Set("error", error);
}

bool JSONQueryRunner::handleLoadFunctionBlock(libvariant::Variant& query, libvariant::Variant& result) {
	result.Set("operation", libvariant::Variant("LOAD"));

	if(query.Contains("name"))  {
		bool operationSuccess = false;
		string name = query.Get("name").AsString();
		if(query.Contains("input")) { // with path
			libvariant::Variant input = query.Get("input");
			if(!input.Contains("metamodel")) {
				LOG(WARNING) << "JSONQueryRunner::handleLoadFunctionBlock: no metamodel defined. Skipping evaluation.";
			}
			if(input.Contains("path")) { // optional path input parameter
				string path = input.Get("path").AsString();
				LOG(DEBUG) << "JSONQueryRunner::handleLoadFunctionBlock: loading block = " << name << " with path = " << path;
				operationSuccess = wm->loadFunctionBlock(name, path);
			}
		} else { // without path
			LOG(DEBUG) << "JSONQueryRunner::handleLoadFunctionBlock: loading block = " << name << " with default path.";
			operationSuccess = wm->loadFunctionBlock(name);
		}

		result.Set("operationSuccess", libvariant::Variant(operationSuccess));
		return operationSuccess;
	} else {
		handleError("Syntax error: Mandatory name field not set in RSGFunctionBlock", result);
	}
	return false;

}

bool JSONQueryRunner::handleUnloadFunctionBlock(libvariant::Variant& query, libvariant::Variant& result) {
	result.Set("operation", libvariant::Variant("UNLOAD"));

	if(query.Contains("name"))  {
		bool operationSuccess = false;
		string name = query.Get("name").AsString();
		operationSuccess = wm->unloadFunctionBlock(name);
		result.Set("operationSuccess", libvariant::Variant(operationSuccess));
		return operationSuccess;
	} else {
		handleError("Syntax error: Mandatory name field not set in RSGFunctionBlock", result);
	}

	return false;
}

bool JSONQueryRunner::handleExecuteFunctionBlock(libvariant::Variant& query, libvariant::Variant& result) {
	result.Set("operation", libvariant::Variant("EXECUTE"));

	if(query.Contains("name"))  {
		bool operationSuccess = false;
		string name = query.Get("name").AsString();
		if(query.Contains("input")) {
			libvariant::Variant input = query.Get("input");
			if(input.Contains("metamodel")) {
				LOG(DEBUG) << "JSONQueryRunner::handleLoadFunctionBlock: metamodel defined. Using model based input.";
				string inputModel = libvariant::Serialize(input, libvariant::SERIALIZE_JSON);
				string outputModel = "";
				operationSuccess = wm->executeFunctionBlock(name, inputModel, outputModel);
				libvariant::Variant outputModelAsJSON = libvariant::Deserialize(outputModel, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON
				result.Set("output", outputModelAsJSON);
			} else {
				LOG(DEBUG) << "JSONQueryRunner::handleLoadFunctionBlock: no metamodel defined. Using Id list instead.";
				std::vector<rsg::Id> inputIds = rsg::JSONTypecaster::getIdsFromJSON(query, "input");
				std::vector<rsg::Id> outputIds;
				operationSuccess = wm->executeFunctionBlock(name, inputIds, outputIds);
				JSONTypecaster::addIdsToJSON(outputIds, result, "output");
			}

		} else {
			handleError("Syntax error: Mandatory input field not set in RSGFunctionBlock", result);
		}
		result.Set("operationSuccess", libvariant::Variant(operationSuccess));
		return operationSuccess;
	} else {
		handleError("Syntax error: Mandatory name field not set in RSGFunctionBlock", result);
	}
	return false;
}

bool JSONQueryRunner::handleConfigureFunctionBlock(libvariant::Variant& query, libvariant::Variant& result) {
	result.Set("operation", libvariant::Variant("CONFIGURE"));

	if(query.Contains("name"))  {
		bool operationSuccess = false;
		string name = query.Get("name").AsString();
		if(query.Contains("input")) {

			std::vector<rsg::Attribute> configuration = JSONTypecaster::getAttributesFromJSON(query,"input");
			operationSuccess = wm->setFunctionBlockConfiguration(name, configuration);

		} else {
			handleError("Syntax error: Mandatory input field not set in RSGFunctionBlock", result);
		}
		result.Set("operationSuccess", libvariant::Variant(operationSuccess));
		return operationSuccess;
	} else {
		handleError("Syntax error: Mandatory name field not set in RSGFunctionBlock", result);
	}
	return false;
}

bool JSONQueryRunner::handleGetMetaModelOfFunctionBlock(libvariant::Variant& query, libvariant::Variant& result) {
	result.Set("operation", libvariant::Variant("GET_METAMODEL"));

	if(query.Contains("name"))  {
		bool operationSuccess = false;
		string name = query.Get("name").AsString();
		string inputMetaModel = "{}";
		string outputMataModel = "{}";

		operationSuccess = wm->getFunctionBlockMetaModel(name, inputMetaModel, outputMataModel);

		libvariant::Variant inputMetaModelAsJSON = libvariant::Deserialize(inputMetaModel, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON
		libvariant::Variant outputMetaModelAsJSON = libvariant::Deserialize(outputMataModel, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON

		/* Concatenat both models */
		libvariant::Variant blockMetaModelAsJSON;
		blockMetaModelAsJSON.Set("inputMetaModel", inputMetaModelAsJSON);
		blockMetaModelAsJSON.Set("outputMetaModel", outputMetaModelAsJSON);
		result.Set("output", blockMetaModelAsJSON);

		result.Set("operationSuccess", libvariant::Variant(operationSuccess));
		return operationSuccess;
	} else {
		handleError("Syntax error: Mandatory name field not set in RSGFunctionBlock", result);
	}
	return false;
}


} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
