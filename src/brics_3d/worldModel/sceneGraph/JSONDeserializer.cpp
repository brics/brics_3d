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

#include "JSONDeserializer.h"
#include <assert.h>

#include "brics_3d/core/HomogeneousMatrix44.h"

namespace brics_3d {
namespace rsg {

JSONDeserializer::JSONDeserializer(WorldModel* wm) :
		wm(wm) {
	mapUnknownParentIdsToRootId = false;
}

JSONDeserializer::~JSONDeserializer() {

}

int JSONDeserializer::write(const char *dataBuffer, int dataLength, int &transferredBytes) {

	libvariant::Variant model = libvariant:: Deserialize(dataBuffer, dataLength, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON
	transferredBytes = dataLength;

	return write(model);
}

int JSONDeserializer::write(std::string data) {

	libvariant::Variant model = libvariant:: Deserialize(data, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON
	return write(model);
}

int JSONDeserializer::write(libvariant::Variant& model) {

	try {

		// Start to parse it
		if(model.Contains("@worldmodeltype")) {

			std::string type = model.Get("@worldmodeltype").AsString();
			if(type.compare("RSGUpdate") == 0) {
				LOG(DEBUG) << "JSONDeserializer: Found a model for an update.";
				handleWorldModelUpdate(model);
			} else if (type.compare("WorldModelAgent") == 0) {
				LOG(DEBUG) << "JSONDeserializer: Found model for a WorldModelAgent.";
				handleWorldModelAgent(model);
			}

		} else {
			LOG(WARNING) << "Top level model type @worldmodeltype does not exist.";
		}


		/* This is indented for debugging purposes only */
		if (model.Contains("@graphtype")) {
			LOG(DEBUG) << "JSONDeserializer: Found a model for a graph primitive";

			Id rootId = 0;
			if (mapUnknownParentIdsToRootId) {
				rootId = wm->getRootNodeId();
			}

			handleGraphPrimitive(model, rootId);
		}

		return 0;

	} catch (std::exception const & e) {
		LOG(ERROR) << "JSONDeserializer: Generic parser error: " << e.what() << std::endl << "Omitting this update.";
		return -1;
	}
}

Id JSONDeserializer::getRootIdFromJSONModel(std::string data) {
	Id rootId = 0; //NiL in case of failure
	libvariant::Variant model = libvariant:: Deserialize(data, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON

	if(model.Contains("@worldmodeltype")) {

		std::string type = model.Get("@worldmodeltype").AsString();
		if (type.compare("WorldModelAgent") == 0) {
			LOG(DEBUG) << "JSONDeserializer: getRootId: Found model for a WorldModelAgent.";

			if(model.Contains("rootNode")) {
				libvariant::Variant rootNode = model.Get("rootNode");
				rootId = rsg::JSONTypecaster::getIdFromJSON(rootNode, "id");
				assert(!rootId.isNil());
				LOG(INFO) << "JSONDeserializer: Extraxted root Id is = " << rootId;
			} else {
				LOG (ERROR) << "JSONDeserializer:  getRootId: WorldModelAgent has no rootID";
				return false;
			}
		}

	} else {
		LOG(WARNING) << "JSONDeserializer: getRootId: Top level model type @worldmodeltype does not exist.";
	}

	return rootId;
}

bool JSONDeserializer::handleWorldModelUpdate(libvariant::Variant& model) {

	if(model.Contains("operation")) {

		string operation = model.Get("operation").AsString();
		if(operation.compare("CREATE") == 0) {

			rsg::Id parentId = rsg::JSONTypecaster::getIdFromJSON(model, "parentId");
			if(parentId.isNil()) {
				LOG(ERROR) << "JSONDeserializer: world model update does not contain a parentId as requiresd for a CREATE operation.";
				return false;
			}

			if(!model.Contains("node")) {
				LOG(ERROR) << "JSONDeserializer: world model update does not contain a node as required for a CREATE operation.";
				return false;
			}
			libvariant::Variant node = model.Get("node");
			return handleGraphPrimitive(node, parentId);

		} else if (operation.compare("CREATE_REMOTE_ROOT_NODE") == 0) {

			if(!model.Contains("node")) {
				LOG(ERROR) << "JSONDeserializer: world model update does not contain a node as required for a CREATE_REMOTE_ROOT_NODE operation.";
				return false;
			}
			libvariant::Variant node = model.Get("node");
			return doAddRemoteRootNode(node);

		} else if (operation.compare("CREATE_PARENT") == 0) {

			rsg::Id parentId = rsg::JSONTypecaster::getIdFromJSON(model, "parentId");
			if(parentId.isNil()) {
				LOG(ERROR) << "JSONDeserializer: world model update does not contain a parentId as requiresd for a CREATE_PARENT operation.";
				return false;
			}

			if(!model.Contains("node")) {
				LOG(ERROR) << "JSONDeserializer: world model update does not contain a node as required for a CREATE_PARENT operation.";
				return false;
			}
			libvariant::Variant node = model.Get("node");
			return doAddParent(node, parentId);

		} else if (operation.compare("UPDATE_ATTRIBUTES") == 0) {

			if(!model.Contains("node")) {
				LOG(ERROR) << "JSONDeserializer: world model update does not contain a node as required for a UPDATE_ATTRIBUTES operation.";
				return false;
			}
			libvariant::Variant node = model.Get("node");
			return doSetNodeAttributes(node);

		} else if (operation.compare("UPDATE_TRANSFORM") == 0) {

			if(!model.Contains("node")) {
				LOG(ERROR) << "JSONDeserializer: world model update does not contain a node as required for a UPDATE_ATTRIBUTES operation.";
				return false;
			}
			libvariant::Variant node = model.Get("node");
			return doSetTransform(node);

		} else if (operation.compare("UPDATE_START") == 0) {
			//TDB
		} else if (operation.compare("UPDATE_END") == 0) {
			//TBD
		} else if (operation.compare("DELETE_NODE") == 0) {

			if(!model.Contains("node")) {
				LOG(ERROR) << "JSONDeserializer: world model update does not contain a node as required for a UPDATE_ATTRIBUTES operation.";
				return false;
			}
			libvariant::Variant node = model.Get("node");
			return doDeleteNode(node);

		} else if (operation.compare("DELETE_PARENT") == 0) {

			rsg::Id parentId = rsg::JSONTypecaster::getIdFromJSON(model, "parentId");
			if(parentId.isNil()) {
				LOG(ERROR) << "JSONDeserializer: world model update does not contain a parentId as requiresd for a DELETE_PARENT operation.";
				return false;
			}

			if(!model.Contains("node")) {
				LOG(ERROR) << "JSONDeserializer: world model update does not contain a node as required for a DELETE_PARENT operation.";
				return false;
			}
			libvariant::Variant node = model.Get("node");
			return doRemoveParent(node, parentId);

		}
	}
			//	       "CREATE",
			//	        "CREATE_REMOTE_ROOT_NODE",
			//		      "CREATE_PARENT",
			//		      "CREATE_PARENT",
			//		      "UPDATE_TRANSFORM",
			//	        "UPDATE_START",
			//	        "UPDATE_END",
			//		      "DELETE_NODE",
			//		      "DELETE_PARENT"
	return false;
}

bool JSONDeserializer::handleWorldModelAgent(libvariant::Variant& model) {

	/* intrinsic world model data */
	if(model.Contains("rootNode")) {
		libvariant::Variant rootNode = model.Get("rootNode");

		/* extract and compare root Id to existin world model */
		Id extractedRootId = rsg::JSONTypecaster::getIdFromJSON(rootNode, "id");
		if( (!extractedRootId.isNil()) && (extractedRootId != wm->getRootNodeId()) ) {
			LOG(ERROR) << "JSONDeserializer: Root Id mismatch. Model file has Id = " << extractedRootId
					<< " while the expected Id is = " << wm->getRootNodeId()
					<< " Did you forget to use getRootIdFromJSONModel at creation of the WorlModel class?";
			return false;
		}

		/* attach attributes */
		std::vector<rsg::Attribute> rootAttributes;
		wm->scene.getNodeAttributes(wm->getRootNodeId(), rootAttributes);
		std::vector<rsg::Attribute> attributes = rsg::JSONTypecaster::getAttributesFromJSON(rootNode);
		rootAttributes.insert(rootAttributes.end(), attributes.begin(), attributes.end());
		wm->scene.setNodeAttributes(wm->getRootNodeId(), rootAttributes);

		/* handle children & connections*/
		handleChilden(rootNode, wm->getRootNodeId());
		handleConnections(rootNode, wm->getRootNodeId());

	} else {
		LOG (ERROR) << "JSONDeserializer: WorldModelAgent has no rootID";
		return false;
	}

	/* extrinisic world model data */


	return true;
}

bool JSONDeserializer::handleGraphPrimitive(libvariant::Variant& atom, rsg::Id parentId) {
	LOG(DEBUG) << "JSONDeserializer: handleGraphPrimitive";
	if(atom.Contains("@graphtype")) {
		string type = atom.Get("@graphtype").AsString();
		LOG(DEBUG) << "JSONDeserializer: Atom has graphtype identifier = " << type;

		if(type.compare("Node") == 0) {

			string semanticContext = "";
			if(atom.Contains("@semanticContext")) {
				semanticContext = atom.Get("@semanticContext").AsString();
				LOG(DEBUG) << "JSONDeserializer: \t\t semanticContext = " << semanticContext;
			}

			/* check for well known semantic conexts */
			if(semanticContext.compare("GeometricNode") == 0) {
				doAddGeometricNode(atom, parentId);

			// else if (...) potentially more to come

			} else { // Default case: a plain node
				doAddNode(atom, parentId);
			}

		} else if (type.compare("Group") == 0) {
			doAddGroup(atom, parentId);
		} else if (type.compare("Connection") == 0) {
			doAddConnection(atom, parentId);
		} else { // ...
			LOG(WARNING) << "JSONDeserializer: Unknown atom graphtype. Skippping it.";
		}
	} else {
		LOG(ERROR) << "JSONDeserializer: Atom has no graphtype identifier";
		return false;
	}

	return true;
}

bool JSONDeserializer::handleChilden(libvariant::Variant& group, rsg::Id parentId) {

	if(group.Contains("childs")) {
		LOG(DEBUG) << "JSONDeserializer: Group has the following childs:";

		libvariant::Variant attributeList = group.Get("childs");
		if (attributeList.IsList()) {
			for (libvariant::Variant::ListIterator i(attributeList.ListBegin()), e(attributeList.ListEnd()); i!=e; ++i) {
				assert(i->Contains("@childtype"));
				string childType = i->Get("@childtype").AsString();
				LOG(DEBUG) << "JSONDeserializer: \t\t type = " << childType;

				if(childType.compare("Child") == 0) {
					libvariant::Variant child = i->Get("child");
					handleGraphPrimitive(child, parentId);
				} else if (childType.compare("ChildId") == 0) {
					// TODO stamps
					assert(i->Contains("childId"));
					rsg::Id childId = rsg::JSONTypecaster::getIdFromJSON(*i, "childId");
					LOG(DEBUG) << "JSONDeserializer: Adding parent -> child relation: " << parentId << " -> " << childId;
					// add parent
					// TODO: cache results
					doAddParent(*i, parentId);
				}
			}
		}
	}

	return true;
}

bool JSONDeserializer::handleConnections(libvariant::Variant& group, rsg::Id parentId) {

	if(group.Contains("connections")) {
			LOG(DEBUG) << "JSONDeserializer: Group has the following connections:";

			libvariant::Variant connectionList = group.Get("connections");
			if (connectionList.IsList()) {
				for (libvariant::Variant::ListIterator i(connectionList.ListBegin()), e(connectionList.ListEnd()); i!=e; ++i) {
					assert(i->Contains("@graphtype"));
					string graphType = i->Get("@graphtype").AsString();
					LOG(DEBUG) << "JSONDeserializer: \t\t type = " << graphType;

					if(graphType.compare("Connection") == 0) {

						string semanticContext = "";
						if(i->Contains("@semanticContext")) {
							semanticContext = i->Get("@semanticContext").AsString();
							LOG(DEBUG) << "JSONDeserializer: \t\t semanticContext = " << semanticContext;
						}

						/* check for well known semantic conexts */
						if(semanticContext.compare("Transform")) {
							doAddTransformNode(*i, parentId);
						} else { // Default case: a plain Conneciton
							doAddConnection(*i, parentId);
						}


					} else {
						LOG(ERROR) << "JSONDeserializer: Connection has no type @graphtype identifier.";
						return false;
					}
				}
			}
		}

	return true;
}

bool JSONDeserializer::doAddNode(libvariant::Variant& group, rsg::Id parentId) {
	LOG(DEBUG) << "JSONDeserializer: doAddNode: type is = " << group.Get("@graphtype").AsString();

	/* Id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(group, "id");
	assert(!id.isNil());

	/* attributes */
	std::vector<rsg::Attribute> attributes = rsg::JSONTypecaster::getAttributesFromJSON(group);

	/* create it */
	wm->scene.addNode(parentId, id, attributes, true);

	return true;
}

bool JSONDeserializer::doAddGroup(libvariant::Variant& group, rsg::Id parentId) {
	LOG(DEBUG) << "JSONDeserializer: doAddGroup: type is = " << group.Get("@graphtype").AsString();

	/* Id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(group, "id");
	assert(!id.isNil());

	/* attributes */
	std::vector<rsg::Attribute> attributes = rsg::JSONTypecaster::getAttributesFromJSON(group);

	/* create it */
	wm->scene.addGroup(parentId, id, attributes, true);

	/* childs (recursion) */
	handleChilden(group, id);

	/* connecitons that are containded in this group */
	handleConnections(group, id);

	return true;
}

bool JSONDeserializer::doAddTransformNode(libvariant::Variant& group,
		rsg::Id parentId) {
	LOG(DEBUG) << "JSONDeserializer doAddTransformNode";

	/*
	 * workaround: current implementation uses a node rather than a connection.
	 */

	/* Id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(group, "id");
	assert(!id.isNil());

	/* attributes */
	std::vector<rsg::Attribute> attributes = rsg::JSONTypecaster::getAttributesFromJSON(group);

	/* Id lists */
//	std::vector<rsg::Id> sourceIds = rsg::JSONTypecaster::getIdsFromJSON(connection, "sourceIds");
//	std::vector<rsg::Id> targetIds = rsg::JSONTypecaster::getIdsFromJSON(connection, "targetIds");
//	assert((sourceIds.size() > 0) || (targetIds.size() > 0)); // there should be at least one

	/* stamps */
	rsg::TimeStamp start = JSONTypecaster::getTimeStampFromJSON(group, "start");
//	rsg::TimeStamp end = JSONTypecaster::getTimeStampFromJSON(connection, "end");

	/* transform data */
	HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44());

	TemporalCache<IHomogeneousMatrix44::IHomogeneousMatrix44Ptr> history;
	JSONTypecaster::getTransformCacheFromJSON(history, group);
	LOG(DEBUG) << "JSONDeserializer doAddTransformNode: transfor cache has size = " << history.size();

	transform = history.getData(history.getLatestTimeStamp());

	/* create it */
	wm->scene.addTransformNode(parentId, id, attributes, transform, history.getLatestTimeStamp(), true);
//	wm->scene.addConnection(parentId, id, attributes, sourceIds, targetIds, start, end, true);


	return true;
}

bool JSONDeserializer::doAddGeometricNode(libvariant::Variant& group,
		rsg::Id parentId) {

	LOG(DEBUG) << "JSONDeserializer: doAddGeometricNode: type is = " << group.Get("@graphtype").AsString();

	Id id = 0;
	vector<Attribute> attributes;
	Shape::ShapePtr shape;
	TimeStamp timeStamp;

	/* Id */
	id = rsg::JSONTypecaster::getIdFromJSON(group, "id");
	assert(!id.isNil());

	/* attributes */
	attributes = rsg::JSONTypecaster::getAttributesFromJSON(group);

	/* shape */
	if(!rsg::JSONTypecaster::getShapeFromJSON(shape, group)) {
		LOG(ERROR) << "JSONDeserializer: doAddGeometricNode: Model  has no Shape";
		return false;
	}


	/* stamp */

	/* create it */
	wm->scene.addGeometricNode(parentId, id, attributes, shape, timeStamp, true);

	return false;
}

bool JSONDeserializer::doAddRemoteRootNode(libvariant::Variant& group) {

	/* Id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(group, "id");
	if(id.isNil()) {
		LOG(ERROR) << "JSONDeserializer: world model update does not contain an id for a doAddRemoteRootNode operation.";
		return false;
	}

	/* attributes */
	std::vector<rsg::Attribute> attributes = rsg::JSONTypecaster::getAttributesFromJSON(group);

	return wm->scene.addRemoteRootNode(id, attributes);

}

bool JSONDeserializer::doAddConnection(libvariant::Variant& connection,
		rsg::Id parentId) {

	LOG(DEBUG) << "JSONDeserializer: doAddConnection";

	/* check for well known semantic conexts */
	string semanticContext = "";
	if(connection.Contains("@semanticContext")) {
		semanticContext = connection.Get("@semanticContext").AsString();
		LOG(DEBUG) << "JSONDeserializer: \t\t semanticContext = " << semanticContext;

		if(semanticContext.compare("Transform") == 0) {
			return doAddTransformNode(connection, parentId);
		}
	}


	/* Id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(connection, "id");
	assert(!id.isNil());

	/* attributes */
	std::vector<rsg::Attribute> attributes = rsg::JSONTypecaster::getAttributesFromJSON(connection);

	/* Id lists */
	std::vector<rsg::Id> sourceIds = rsg::JSONTypecaster::getIdsFromJSON(connection, "sourceIds");
	std::vector<rsg::Id> targetIds = rsg::JSONTypecaster::getIdsFromJSON(connection, "targetIds");
	assert((sourceIds.size() > 0) || (targetIds.size() > 0)); // there should be at least one

	/* stamps */
	rsg::TimeStamp start = JSONTypecaster::getTimeStampFromJSON(connection, "start");
	rsg::TimeStamp end = JSONTypecaster::getTimeStampFromJSON(connection, "end");

	/* create it */
	return wm->scene.addConnection(parentId, id, attributes, sourceIds, targetIds, start, end, true);
}

bool JSONDeserializer::doSetNodeAttributes(libvariant::Variant& group) {

	/* Id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(group, "id");
	if(id.isNil()) {
		LOG(ERROR) << "JSONDeserializer: world model update does not contain an id for a doSetNodeAttributes operation.";
		return false;
	}

	/* attributes */
	std::vector<rsg::Attribute> attributes = rsg::JSONTypecaster::getAttributesFromJSON(group);

	/* time stamp with fall back to current time */
	rsg::TimeStamp attributesTimeStamp;
	string stampTag = "attributesTimeStamp";
	if(group.Contains(stampTag)) {
		attributesTimeStamp = rsg::JSONTypecaster::getTimeStampFromJSON(group, stampTag);
	} else {
		attributesTimeStamp = wm->now();
	}

	return wm->scene.setNodeAttributes(id, attributes, attributesTimeStamp);

	return false;
}

bool JSONDeserializer::doSetTransform(libvariant::Variant& group) {

	LOG(DEBUG) << "JSONDeserializer doSetTransform";


	/* Id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(group, "id");
	if(id.isNil()) {
		LOG(ERROR) << "JSONDeserializer: world model update does not contain an id for a doSetTransform operation.";
		return false;
	}

	/* transform data */
	HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44());

	TemporalCache<IHomogeneousMatrix44::IHomogeneousMatrix44Ptr> history;
	JSONTypecaster::getTransformCacheFromJSON(history, group);
	LOG(DEBUG) << "JSONDeserializer doSetTransform: transform cache has size = " << history.size();

	transform = history.getData(history.getLatestTimeStamp());

	/* create it */
	return wm->scene.setTransform(id, transform, history.getLatestTimeStamp());

}

bool JSONDeserializer::doDeleteNode(libvariant::Variant& group) {

	/* Id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(group, "id");
	if(id.isNil()) {
		LOG(ERROR) << "JSONDeserializer: world model update does not contain an id for a doDeleteNode operation.";
		return false;
	}

	return wm->scene.deleteNode(id);
}

bool JSONDeserializer::doAddParent(libvariant::Variant& group,
		rsg::Id parentId) {

	/* id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(group, "childId");

	/* stamps */
	rsg::TimeStamp start = JSONTypecaster::getTimeStampFromJSON(group, "start");
	rsg::TimeStamp end = JSONTypecaster::getTimeStampFromJSON(group, "end");

	return wm->scene.addParent(id, parentId);
}

bool JSONDeserializer::doRemoveParent(libvariant::Variant& group,
		rsg::Id parentId) {

	/* Id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(group, "id");
	if(id.isNil()) {
		LOG(ERROR) << "JSONDeserializer: world model update does not contain an id for a doDeleteNode operation.";
		return false;
	}

	return wm->scene.removeParent(id, parentId);
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
