/**
 * @file 
 * AttributeFinder.cpp
 *
 * @date: Oct 27, 2011
 * @author: sblume
 */

#include "AttributeFinder.h"
#include "core/Logger.h"

namespace BRICS_3D {

namespace RSG {

AttributeFinder::AttributeFinder() : INodeVisitor(downwards) {
	reset();
}

AttributeFinder::~AttributeFinder() {

}

void AttributeFinder::visit(Node* node) {
	assert (node != 0);
	std::cout << "Visiting node with ID: " << node->getId() << std::endl;

	/* check for duplicates (possibly causesd by _graph_ traversal) */
	for (unsigned int i = 0; i < static_cast<unsigned int>(matchingNodes.size()); ++i) {
		if (matchingNodes[i] == node) {
			return; //just stop
		}
	}

	/* check if attributes match */
	bool attributesMatch = false;
	for (unsigned int i = 0; i < static_cast<unsigned int>(queryAttributes.size()); ++i) {
		attributesMatch = false;
		if (attributeListContainsAttribute(node->getAttributes(), queryAttributes[i]) == false) {
			break;
		}
		attributesMatch = true; // returns true only when all loops are sucesfully/completely traversed
	}

	if (attributesMatch == true) {
		std::cout << "	Adding node as it has: ";
		for (unsigned int i = 0; i < static_cast<unsigned int>(queryAttributes.size()); ++i) {
			std::cout << " " << queryAttributes[i];
		}
		std::cout << std::endl;

		matchingNodes.push_back(node);
	}
}

void AttributeFinder::visit(Group* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void AttributeFinder::visit(Transform* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void AttributeFinder::visit(GeometricNode* node){
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
}

void AttributeFinder::reset() {
	matchingNodes.clear();
}

}

}

/* EOF */
