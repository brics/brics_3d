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

#include "NodeHashTraverser.h"
#include "NodeHash.h"

namespace brics_3d {
namespace rsg {

const std::string NodeHashTraverser::NIL = "NO-HASH-FOR-ID";

NodeHashTraverser::NodeHashTraverser() : INodeVisitor(custom) {
	reset();
}

NodeHashTraverser::~NodeHashTraverser() {

}

void NodeHashTraverser::visit(Node* node) {
	string nodeHash = NodeHash::nodeToHash(node, useNodeIds);
	hashLookUpTable.insert(std::make_pair(node->getId(), nodeHash));
	LOG(DEBUG) << "NodeHashTraverser: hash for Node:  " << node->getId() << " = " << nodeHash;
}

void NodeHashTraverser::visit(Group* node) {
	std::vector<std::string> hashes;
	for(unsigned i = 0; i < node->getNumberOfChildren(); ++i) { // recursively go down the graph structure
		// calculate hashes recursively
		node->getChild(i)->accept(this);

		// collect all hashes of children in one set
		string hash = getHashById(node->getChild(i)->getId());
		hashes.push_back(hash);
	}

	// calculate hash part one: attributes an id this node
	string nodeHash = NodeHash::nodeToHash(node, useNodeIds);
	hashes.push_back(nodeHash);

	// calculate hash part two: combine node hash with the child hashes to a combined group hash
	string groupHash = NodeHash::sortStringsAndHash(hashes);

	// store it in global hash map
	hashLookUpTable.insert(std::make_pair(node->getId(), groupHash));
	LOG(DEBUG) << "NodeHashTraverser: hash for Group: " << node->getId() << " = " << groupHash;
}

void NodeHashTraverser::visit(Transform* node) {
	this->visit(dynamic_cast<Group*>(node)); //just feed forward to be handled as Group
}

void NodeHashTraverser::visit(GeometricNode* node) {
	this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as Node
}

void NodeHashTraverser::visit(Connection* connection) {
	std::vector<std::string> hashes;

	// collect all hashes of source and target IDs in one set
	for(unsigned i = 0; i < connection->getNumberOfSourceNodes(); ++i) { // get "neighboring" nodes.
		string hash = getHashById(connection->getSourceNode(i)->getId());
		hashes.push_back(hash);
	}

	for(unsigned i = 0; i < connection->getNumberOfTargetNodes(); ++i) { // get "neighboring" nodes.
		string hash = getHashById(connection->getTargetNode(i)->getId());
		hashes.push_back(hash);
	}

	// calculate hash part one: attributes an id this node
	string nodeHash = NodeHash::nodeToHash(connection, useNodeIds);
	hashes.push_back(nodeHash);

	// calculate hash part two: combine node hash with the source/target hashes to a combined connection hash
	string connectionHash = NodeHash::sortStringsAndHash(hashes);

	hashLookUpTable.insert(std::make_pair(connection->getId(), connectionHash));
	LOG(DEBUG) << "NodeHashTraverser: hash for Connection: " << connection->getId() << " = " << connectionHash;
}

void NodeHashTraverser::reset(bool useNodeIds) {
	this->useNodeIds = useNodeIds;
	hashLookUpTable.clear();
}

std::string NodeHashTraverser::getHashById(Id id) {
	hashIterator = hashLookUpTable.find(id);
	if (hashIterator != hashLookUpTable.end()) {
		return hashIterator->second;
	}

	LOG(ERROR) << "NodeHashTraverser: Hash look up failed for ID " << id;
	return NIL;

}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
