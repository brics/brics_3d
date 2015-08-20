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

/*
 * This example shows the usage of the json parser tools.
 */

#include <brics_3d/core/Logger.h>
#include <brics_3d/worldModel/WorldModel.h>

#include <Variant/Variant.h>
#include <stdio.h>

// Import Variant from libvariant
using libvariant::Variant;


using brics_3d::Logger;
using namespace brics_3d;

bool handleGraphPrimitive(Variant& atom, rsg::Id parentId);
bool handleNode(Variant& node, rsg::Id parentId);
bool handleGroup(Variant& group, rsg::Id parentId);
brics_3d::rsg::Id getId(Variant& node, string idTag);
std::vector<rsg::Attribute> getAttributes (Variant& node);


int main(int argc, char **argv) {

	Logger::setMinLoglevel(Logger::LOGDEBUG);
	LOG(INFO) << " JSON parser test.";

	if (argc != 2) {
		printf("Usage: %s input_file\n", *argv);
		return 1;
	}

	const char *fileName = argv[1];
    Variant model = libvariant::DeserializeJSONFile(fileName);

	// Pretty print to stdout
	libvariant::SerializeJSON(stdout, model, true);

	if(model.Contains("type")) {
		LOG(INFO) << "Top level model type exists.";
	}

	handleGraphPrimitive(model, 0);

	return 0;
}

/**
 * Inferes the type and calls the respecive handleXY
 * @param atom
 * @param parentId
 * @return
 */
bool handleGraphPrimitive(Variant& atom, rsg::Id parentId) {
	if(atom.Contains("type")) {
		LOG(DEBUG) << "Atom has type identifier";
		string type = atom.Get("type").AsString();
		LOG(DEBUG) << "\t type = " << type;

		if(type.compare("Node") == 0) {
			handleNode(atom, parentId);
		} else if (type.compare("Group") == 0) {
			handleGroup(atom, parentId);
		} else { // ...
			LOG(WARNING) << "Unknown atom type. Skippping it.";
		}
	} else {
		LOG(ERROR) << "Atom has no type identifier";
		return false;
	}

	return true;
}

bool handleNode(Variant& node, rsg::Id parentId) {
	LOG(DEBUG) << "handleNode: type is" << node.Get("type").AsString();

	/* Id */
	rsg::Id id = getId(node, "id");
	assert(!id.isNil());

	/* attributes */
	std::vector<rsg::Attribute> attributes = getAttributes(node);

	/* create it */


	return true;
}


/**
 * Handle paring of a "group" primitive.
 * @param group The JSON version of a group
 * @param parentId For recursion we need to pass the parent Id as extracted from the group on the "upper" level
 *        in the graph.
 * @return true on success
 */
bool handleGroup(Variant& group, rsg::Id parentId) {
	LOG(DEBUG) << "handleGroup: type is" << group.Get("type").AsString();

	/* Id */
	rsg::Id id = getId(group, "id");
	assert(!id.isNil());

	/* attributes */
	std::vector<rsg::Attribute> attributes = getAttributes(group);

	/* childs (recursion) */
	if(group.Contains("childs")) {
		LOG(DEBUG) << "Group has the following childs:";

		Variant attributeList = group.Get("childs");
		if (attributeList.IsList()) {
			for (Variant::ListIterator i(attributeList.ListBegin()), e(attributeList.ListEnd()); i!=e; ++i) {
				assert(i->Contains("type"));
				string childType = i->Get("type").AsString();
				LOG(DEBUG) << " type = " << childType;

				if(childType.compare("Child") == 0) {
					Variant child = i->Get("child");
					handleGraphPrimitive(child, id);
				} else if (childType.compare("ChildId") == 0) {
					// TODO stamps
					assert(i->Contains("childId"));
					rsg::Id childId = getId(*i, "childId");
					LOG(DEBUG) << "Adding parent -> child relation: " << id << " -> " << childId;
					// add parent
				}
			}
		}
	}
	return true;
}

brics_3d::rsg::Id getId(Variant& node, string idTag) {
	rsg::Id id = 0; //NiL

	if(node.Contains(idTag)) { // required
		id.fromString(node.Get(idTag).AsString());
		assert(!id.isNil());
		LOG(DEBUG) << "Id " << idTag << " is = " << id;
	} else {
		LOG(ERROR) << "Can not parse model. No node Id " << idTag << " specified.";
	}

	return id;
}

std::vector<rsg::Attribute> getAttributes (Variant& node) {
	std::vector<rsg::Attribute> attributes;
	attributes.clear();

	if(node.Contains("attributes")) {
		LOG(DEBUG) << "Node has the following attributes:";

		Variant attributeList = node.Get("attributes");
		if (attributeList.IsList()) {
			for (Variant::ListIterator i(attributeList.ListBegin()), e(attributeList.ListEnd()); i!=e; ++i) {
				assert(i->Contains("key"));
				assert(i->Contains("value"));
				LOG(DEBUG) << "\t( " << i->Get("key").AsString() << ", " << i->Get("value").AsString() << " )";
				attributes.push_back(rsg::Attribute(i->Get("key").AsString(), i->Get("value").AsString()));
			}
		}
	}

	return attributes;
}
