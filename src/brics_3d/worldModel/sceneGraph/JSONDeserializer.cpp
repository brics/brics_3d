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

namespace brics_3d {
namespace rsg {

JSONDeserializer::JSONDeserializer(WorldModel* wm) :
		wm(wm) {
	mapUnknownParentIdsToRootId = false;
}

JSONDeserializer::~JSONDeserializer() {

}

int JSONDeserializer::write(const char *dataBuffer, int dataLength, int &transferredBytes) {
	return -1;
}

int JSONDeserializer::write(std::string data) {

	libvariant::Variant model = libvariant:: Deserialize(data, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON

	// Start to parse it
	if(model.Contains("@worldmodeltype")) {

		std::string type = model.Get("@worldmodeltype").AsString();
		if(type.compare("RSGUpdate") == 0) {
			LOG(DEBUG) << "JSONDeserializer: Found a model for an update.";
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
		LOG(DEBUG) << "JSONDeserializer: Atom has graphtype identifier = " << type; ;

		if(type.compare("Node") == 0) {
			doAddNode(atom, parentId);
		} else if (type.compare("Group") == 0) {
			doAddGroup(atom, parentId);
		} else if (type.compare("Connection")) {
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

						// TODO

						doAddConnection(*i, parentId);
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
	return false;
}

bool JSONDeserializer::doAddGeometricNode(libvariant::Variant& group,
		rsg::Id parentId) {
	return false;
}

bool JSONDeserializer::doAddRemoteRootNode(libvariant::Variant& group,
		rsg::Id parentId) {
	return false;
}

bool JSONDeserializer::doAddConnection(libvariant::Variant& connection,
		rsg::Id parentId) {

	LOG(DEBUG) << "JSONDeserializer: doAddConnection";

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
	wm->scene.addConnection(parentId, id, attributes, sourceIds, targetIds, start, end, true);

	return false;
}

bool JSONDeserializer::doSetNodeAttributes(libvariant::Variant& group,
		rsg::Id parentId) {
	return false;
}

bool JSONDeserializer::doSetTransform(libvariant::Variant& group,
		rsg::Id parentId) {
	return false;
}

bool JSONDeserializer::doDeleteNode(libvariant::Variant& group,
		rsg::Id parentId) {
	return false;
}

bool JSONDeserializer::doAddParent(libvariant::Variant& group,
		rsg::Id parentId) {
	return false;
}

bool JSONDeserializer::doRemoveParent(libvariant::Variant& group,
		rsg::Id parentId) {
	return false;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
