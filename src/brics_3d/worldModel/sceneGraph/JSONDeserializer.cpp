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
	if(!model.Contains("type")) {
		LOG(ERROR) << "Top level model type does not exist.";
		return -1;
	}

	std::string type = model.Get("type").AsString();
	if(type.compare("RSGUpdate") == 0) {
		LOG(DEBUG) << "JSONDeserializer: Found a model for an update.";
	} else if (type.compare("WorldModelAgent") == 0) {
		LOG(DEBUG) << "JSONDeserializer: Found model for a WorldModelAgent.";
	} else {
		// This is indented for debugging purposes only
		LOG(DEBUG) << "JSONDeserializer: Found a model for a graph primitive";

		Id rootId = 0;
		if (mapUnknownParentIdsToRootId) {
			rootId = wm->getRootNodeId();
		}

		handleGraphPrimitive(model, rootId);
	}

	return 0;
}

bool JSONDeserializer::handleGraphPrimitive(libvariant::Variant& atom, rsg::Id parentId) {
	LOG(DEBUG) << "JSONDeserializer: handleGraphPrimitive";
	if(atom.Contains("type")) {
		string type = atom.Get("type").AsString();
		LOG(DEBUG) << "JSONDeserializer: Atom has type identifier = "<< type; ;

		if(type.compare("Node") == 0) {
			doAddNode(atom, parentId);
		} else if (type.compare("Group") == 0) {
			doAddGroup(atom, parentId);
		} else { // ...
			LOG(WARNING) << "JSONDeserializer: Unknown atom type. Skippping it.";
		}
	} else {
		LOG(ERROR) << "SONDeserializer: Atom has no type identifier";
		return false;
	}

	return true;
}

bool JSONDeserializer::doAddNode(libvariant::Variant& group, rsg::Id parentId) {
	LOG(DEBUG) << "JSONDeserializer: doAddNode: type is = " << group.Get("type").AsString();

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
	LOG(DEBUG) << "JSONDeserializer: doAddGroup: type is = " << group.Get("type").AsString();

	/* Id */
	rsg::Id id = rsg::JSONTypecaster::getIdFromJSON(group, "id");
	assert(!id.isNil());

	/* attributes */
	std::vector<rsg::Attribute> attributes = rsg::JSONTypecaster::getAttributesFromJSON(group);

	/* create it */
	wm->scene.addGroup(parentId, id, attributes, true);

	/* childs (recursion) */
	if(group.Contains("childs")) {
		LOG(DEBUG) << "JSONDeserializer: Group has the following childs:";

		libvariant::Variant attributeList = group.Get("childs");
		if (attributeList.IsList()) {
			for (libvariant::Variant::ListIterator i(attributeList.ListBegin()), e(attributeList.ListEnd()); i!=e; ++i) {
				assert(i->Contains("type"));
				string childType = i->Get("type").AsString();
				LOG(DEBUG) << "JSONDeserializer: \t\t type = " << childType;

				if(childType.compare("Child") == 0) {
					libvariant::Variant child = i->Get("child");
					handleGraphPrimitive(child, id);
				} else if (childType.compare("ChildId") == 0) {
					// TODO stamps
					assert(i->Contains("childId"));
					rsg::Id childId = rsg::JSONTypecaster::getIdFromJSON(*i, "childId");
					LOG(DEBUG) << "JSONDeserializer: Adding parent -> child relation: " << id << " -> " << childId;
					// add parent
				}
			}
		}
	}

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

bool JSONDeserializer::doAddConnection(libvariant::Variant& group,
		rsg::Id parentId) {
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
