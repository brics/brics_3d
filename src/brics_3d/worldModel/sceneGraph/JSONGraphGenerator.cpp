/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2017, KU Leuven
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

#include "JSONGraphGenerator.h"

namespace brics_3d {
namespace rsg {

JSONGraphGenerator::JSONGraphGenerator() : INodeVisitor(custom) {
	reset();
}

JSONGraphGenerator::~JSONGraphGenerator() {
	reset();
}

void JSONGraphGenerator::visit(Node* node) {
	LOG(DEBUG) << "JSONGraphGenerator: adding a Node-" << node->getId().toString();

	try {

		/* the actual graph primitive */
		libvariant::Variant jsonNode;
		jsonNode.Set("@graphtype", libvariant::Variant("Node"));
		JSONTypecaster::addIdToJSON(node->getId(), jsonNode, "id");
		JSONTypecaster::addAttributesToJSON(node->getAttributes(), jsonNode);
		TimeStamp attributesTimeStamp = node->getAttributesTimeStamp();
		JSONTypecaster::addTimeStampToJSON(attributesTimeStamp, jsonNode, "attributesTimeStamp");

		/* store it for later assembly */
		jsonLookUpTable.insert(std::make_pair(node->getId(), jsonNode));

	} catch (std::exception e) {
		LOG(ERROR) << "JSONGraphGenerator addNode: Cannot create a JSON serialization. Exception = " << std::endl << e.what();
	}
}

void JSONGraphGenerator::visit(Group* node) {

	/* recursively go down the graph structure */
	for(unsigned i = 0; i < node->getNumberOfChildren(); ++i) {
		// calculate hashes recursively
		node->getChild(i)->accept(this);
	}

	/* during unwinding of the recusrion stack: build up the JSON model */
	LOG(DEBUG) << "JSONGraphGenerator: adding a Group-" << node->getId().toString();
	try {

		/* the actual graph primitive */
		libvariant::Variant jsonNode;
		jsonNode.Set("@graphtype", libvariant::Variant("Group"));
		JSONTypecaster::addIdToJSON(node->getId(), jsonNode, "id");
		JSONTypecaster::addAttributesToJSON(node->getAttributes(), jsonNode);
		TimeStamp attributesTimeStamp = node->getAttributesTimeStamp();
		JSONTypecaster::addTimeStampToJSON(attributesTimeStamp, jsonNode, "attributesTimeStamp");


		/* attach all childs */
		libvariant::Variant childs(libvariant::VariantDefines::ListType);
		for(unsigned i = 0; i < node->getNumberOfChildren(); ++i) { // FIXME: already traversed childs
			libvariant::Variant child;
			child.Set("@childtype", libvariant::Variant("Child"));
			child.Set("child", getJSONById(node->getChild(i)->getId()));
			childs.Append(child);
		}
		jsonNode.Set("childs", childs);

		/* store it for later assembly */
		jsonLookUpTable.insert(std::make_pair(node->getId(), jsonNode));

	} catch (std::exception e) {
		LOG(ERROR) << "JSONGraphGenerator addNode: Cannot create a JSON serialization. Exception = " << std::endl << e.what();
	}

}

void JSONGraphGenerator::visit(Transform* node) {

	/* recursively go down the graph structure */
	for(unsigned i = 0; i < node->getNumberOfChildren(); ++i) {
		// calculate hashes recursively
		node->getChild(i)->accept(this);
	}

	/* during unwinding of the recusrion stack: build up the JSON model */
	LOG(DEBUG) << "JSONGraphGenerator: adding a Transform-" << node->getId().toString();
	try {


		/* the actual graph primitive */
		libvariant::Variant jsonNode;
		jsonNode.Set("@graphtype", libvariant::Variant("Connection"));
		jsonNode.Set("@semanticContext", libvariant::Variant("Transform"));
		JSONTypecaster::addIdToJSON(node->getId(), jsonNode, "id");
		JSONTypecaster::addAttributesToJSON(node->getAttributes(), jsonNode);
		TimeStamp attributesTimeStamp = node->getAttributesTimeStamp();
		JSONTypecaster::addTimeStampToJSON(attributesTimeStamp, jsonNode, "attributesTimeStamp");
		TemporalCache<IHomogeneousMatrix44::IHomogeneousMatrix44Ptr> history = node->getHistory();
		JSONTypecaster::addTransformCacheToJSON(history, jsonNode);

//		/* assamble it */
//		graphUpdate.Set("node", node);

	} catch (std::exception e) {
		LOG(ERROR) << "JSONGraphGenerator addNode: Cannot create a JSON serialization. Exception = " << std::endl << e.what();
	}

}

void JSONGraphGenerator::visit(GeometricNode* node) {
	LOG(DEBUG) << "JSONGraphGenerator: adding a Node-" << node->getId().toString();

	try {

		/* the actual graph primitive */
		libvariant::Variant jsonNode;
		jsonNode.Set("@graphtype", libvariant::Variant("Node"));
		jsonNode.Set("@semanticContext", libvariant::Variant("GeometricNode"));
		JSONTypecaster::addIdToJSON(node->getId(), jsonNode, "id");
		JSONTypecaster::addAttributesToJSON(node->getAttributes(), jsonNode);
		TimeStamp attributesTimeStamp = node->getAttributesTimeStamp();
		JSONTypecaster::addTimeStampToJSON(attributesTimeStamp, jsonNode, "attributesTimeStamp");
		brics_3d::rsg::Shape::ShapePtr shape = node->getShape();
		JSONTypecaster::addShapeToJSON(shape, jsonNode, "geometry");
		JSONTypecaster::addTimeStampToJSON(node->getTimeStamp(), jsonNode, "timeStamp");
		jsonNode.Set("unit", libvariant::Variant("m")); // TODO; better part of geometry?!?

		/* store it for later assembly */
		jsonLookUpTable.insert(std::make_pair(node->getId(), jsonNode));

	} catch (std::exception e) {
		LOG(ERROR) << "JSONGraphGenerator addNode: Cannot create a JSON serialization. Exception = " << std::endl << e.what();
	}
}

void JSONGraphGenerator::visit(Connection* connection) {

}

void JSONGraphGenerator::reset() {
	jsonLookUpTable.clear();
	model.Clear();
}

std::string JSONGraphGenerator::getJSON() {
	return "false";
}

libvariant::Variant& JSONGraphGenerator::getJSONModel() {
	return model;
}

libvariant::Variant JSONGraphGenerator::getJSONById(Id id) {
	jsonIterator = jsonLookUpTable.find(id);
	if (jsonIterator != jsonLookUpTable.end()) {
		return jsonIterator->second;
	}

	LOG(ERROR) << "JSONGraphGenerator: JSON look up failed for ID " << id;
	return libvariant::Variant(libvariant::VariantDefines::NullType);
}


} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
