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
	LOG(INFO) << "Visiting node with ID: " << node->getId();
//	currentPath.insert(currentPath.begin(), node);
//	if (node->getNumberOfParents() == 0) { //root reached so collect data in new path
//		if (static_cast<unsigned int>(nodePaths.size()) == 0u) { //remove the "caller's" pointer
//				currentPath.pop_back();
//		}
//		nodePaths.push_back(currentPath);
//		currentPath.clear();
//	}
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
