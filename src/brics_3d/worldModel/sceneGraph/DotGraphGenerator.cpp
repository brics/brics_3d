/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
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

#include "DotGraphGenerator.h"
#include "brics_3d/core/Logger.h"

#include <boost/algorithm/string/replace.hpp>

namespace brics_3d {

namespace rsg {

DotGraphGenerator::DotGraphGenerator() {
	reset();
}

DotGraphGenerator::~DotGraphGenerator() {
	reset();
}

void DotGraphGenerator::reset() {
	alreadyVisitedNodes.clear();
	nodes.str("");
	edges.str("");
	dotGraph.str("");
}

void DotGraphGenerator::visit(Node* node){
	assert (node != 0);

	vector<Node*>::iterator visitedNodes;
	visitedNodes = find (alreadyVisitedNodes.begin(), alreadyVisitedNodes.end(), node);

	if (visitedNodes == alreadyVisitedNodes.end()) { // if not in list: insert and handle node
		//LOG(DEBUG) << " DotGraphGenerator::visit(Node* node): visiting: " << node->getId();
		doHandleNode(node);
		alreadyVisitedNodes.push_back(node);
	}
}

void DotGraphGenerator::visit(Group* node){
	assert (node !=0);
	doHandleEdges(node); //all edges to children will be processed
	this->visit(dynamic_cast<Node*>(node)); //and feed forward to be handled as node
}

void DotGraphGenerator::visit(Transform* node){
	//this->visit(dynamic_cast<Group*>(node)); //feed forward to be handled as group TODO a diffrent handling?

	assert (node != 0);
	doHandleEdges(node);

	vector<Node*>::iterator visitedNodes;
	visitedNodes = find (alreadyVisitedNodes.begin(), alreadyVisitedNodes.end(), node);

	if (visitedNodes == alreadyVisitedNodes.end()) { // if not in list: insert and handle node
		doHandleTransform(node);
		alreadyVisitedNodes.push_back(node);
	}
}

void DotGraphGenerator::visit(GeometricNode* node){
	//this->visit(dynamic_cast<Node*>(node)); //just feed forward to be handled as node
	assert (node != 0);

	vector<Node*>::iterator visitedNodes;
	visitedNodes = find (alreadyVisitedNodes.begin(), alreadyVisitedNodes.end(), node);

	if (visitedNodes == alreadyVisitedNodes.end()) { // if not in list: insert and handle node
		doHandleGeometricNode(node);
		alreadyVisitedNodes.push_back(node);
	}
}

void DotGraphGenerator::visit(Connection* connection) {
	vector<Node*>::iterator visitedNodes;
	visitedNodes = find (alreadyVisitedNodes.begin(), alreadyVisitedNodes.end(), connection);

	if (visitedNodes == alreadyVisitedNodes.end()) { // if not in list: insert and handle node
		LOG(DEBUG) << " DotGraphGenerator::visit(Node* node): Connection found";
		doHandleConnection(connection);
		alreadyVisitedNodes.push_back(connection);
	}
}


void DotGraphGenerator::doHandleNode(Node* node) {
	assert (node !=0);
//	LOG(DEBUG) << "Adding node " << node->getId() << " to dot graph.";

	//Each node should add a line like this: 2 [label = "ID [2]\n(name = point_cloud_1)\n"];

	std::stringstream aggregatedAttributes;
	vector<Attribute> attributeList = node->getAttributes();
	for (unsigned int i = 0; i < static_cast<unsigned int>(attributeList.size()); ++i) {
		aggregatedAttributes << attributeList[i];
		aggregatedAttributes << "\\n";
	}

    if(config.abbreviateIds) {
    	nodes << "\"";
    	nodes << uuidToUnsignedInt(node->getId());
    	nodes << "\"";
        nodes << " [label = \"";
    	nodes << "ID [" << uuidToUnsignedInt(node->getId()) << "]\\n";
    } else {
    	nodes << "\"";
    	nodes << node->getId();
    	nodes << "\"";
        nodes << " [label = \"";
    	nodes << "ID [" << node->getId() << "]\\n";
	}
    nodes << stringify(aggregatedAttributes.str());
    nodes << "\"]";
	nodes << ";" << std::endl;
}

void DotGraphGenerator::doHandleConnection(Connection* connection) {
	assert (connection !=0);
//	LOG(DEBUG) << "Adding node " << node->getId() << " to dot graph.";

	//Each node should add a line like this: 2 [label = "ID [2]\n(name = point_cloud_1)\n"];

	std::stringstream aggregatedAttributes;
	vector<Attribute> attributeList = connection->getAttributes();
	for (unsigned int i = 0; i < static_cast<unsigned int>(attributeList.size()); ++i) {
		aggregatedAttributes << attributeList[i];
		aggregatedAttributes << "\\n";
	}

    if(config.abbreviateIds) {
    	nodes << "\"";
    	nodes << uuidToUnsignedInt(connection->getId());
    	nodes << "\"";
        nodes << " [label = \"";
    	nodes << "ID [" << uuidToUnsignedInt(connection->getId()) << "]\\n";
    } else {
    	nodes << "\"";
    	nodes << connection->getId();
    	nodes << "\"";
        nodes << " [label = \"";
    	nodes << "ID [" << connection->getId() << "]\\n";
	}
	nodes << "Connection\\n";
    nodes << stringify(aggregatedAttributes.str());
    nodes << "\"";
    nodes << "style=\"rounded,filled\" shape=diamond ";
//    nodes << "style=\"filled\" ";
    nodes << "fillcolor=lightblue ";
    nodes <<"]";
	nodes << ";" << std::endl;

	/* Add further edges for in going arcs */
	for (unsigned int sourceIndex = 0; sourceIndex < connection->getNumberOfSourceNodes(); ++sourceIndex) {
		if(config.abbreviateIds) {
			edges << "\"" << uuidToUnsignedInt(connection->getSourceNode(sourceIndex)->getId()) << "\"" << " -> " << "\"" << uuidToUnsignedInt(connection->getId()) << "\"" << " [ style=dashed, label=\"in\" ] " << ";" << std::endl;
		} else {
			edges << "\"" << connection->getSourceNode(sourceIndex)->getId() << "\"" << " -> " << "\"" << connection->getId() << "\"" << " [ style=dashed, label=\"in\" ] " << ";" << std::endl;
		}
	}
	/* Add further edges for out going arcs */
	for (unsigned int targetIndex = 0; targetIndex < connection->getNumberOfTargetNodes(); ++targetIndex) {
		if(config.abbreviateIds) {
			edges << "\"" << uuidToUnsignedInt(connection->getId())  << "\"" << " -> " << "\"" << uuidToUnsignedInt(connection->getTargetNode(targetIndex)->getId()) << "\"" << " [ style=dashed, label=\"out\" ] " << ";" << std::endl;
		} else {
			edges << "\"" << connection->getId() << "\"" << " -> " << "\"" << connection->getTargetNode(targetIndex)->getId() << "\"" << " [ style=dashed, label=\"out\" ] " << ";" << std::endl;
		}
	}

}

void DotGraphGenerator::doHandleTransform(Transform* node) {
	assert (node !=0);
	std::stringstream aggregatedAttributes;
	vector<Attribute> attributeList = node->getAttributes();
	for (unsigned int i = 0; i < static_cast<unsigned int>(attributeList.size()); ++i) {
		aggregatedAttributes << attributeList[i];
		aggregatedAttributes << "\\n";
	}
	node->getUpdateCount();
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr currentTransform;
	currentTransform = node->getLatestTransform();
	const double* matrixPtr = currentTransform->getRawData();
	double x = matrixPtr[12];
	double y = matrixPtr[13];
	double z = matrixPtr[14];


    if(config.abbreviateIds) {
    	nodes << "\"";
    	nodes << uuidToUnsignedInt(node->getId());
    	nodes << "\"";
        nodes << " [label = \"";
    	nodes << "ID [" << uuidToUnsignedInt(node->getId()) << "]\\n";
    } else {
    	nodes << "\"";
    	nodes << node->getId();
    	nodes << "\"";
        nodes << " [label = \"";
    	nodes << "ID [" << node->getId() << "]\\n";
	}
    nodes << "T = (" << x << ", " << y <<", " << z << ")\\n";
    nodes << "Updates: " << node->getUpdateCount() << "\\n";
    nodes << stringify(aggregatedAttributes.str());
    nodes << "\"";
//    nodes << "style=\"rounded,filled\", shape=diamond";
//    nodes << "shape=circle";
    nodes << "style=\"filled\" ";
    nodes << "fillcolor=yellow ";
    nodes <<"]";
	nodes << ";" << std::endl;
}

void DotGraphGenerator::doHandleGeometricNode(GeometricNode* node) {
	assert (node !=0);

	std::stringstream aggregatedAttributes;
	vector<Attribute> attributeList = node->getAttributes();
	for (unsigned int i = 0; i < static_cast<unsigned int>(attributeList.size()); ++i) {
		aggregatedAttributes << attributeList[i];
		aggregatedAttributes << "\\n";
	}

    if(config.abbreviateIds) {
    	nodes << "\"";
    	nodes << uuidToUnsignedInt(node->getId());
    	nodes << "\"";
        nodes << " [label = \"";
    	nodes << "ID [" << uuidToUnsignedInt(node->getId()) << "]\\n";
    } else {
    	nodes << "\"";
    	nodes << node->getId();
    	nodes << "\"";
        nodes << " [label = \"";
    	nodes << "ID [" << node->getId() << "]\\n";
	}
    nodes << stringify(aggregatedAttributes.str());
    nodes << "\"";
    nodes << "style=\"filled\" ";
//    nodes << "fillcolor=green ";
    nodes << "fillcolor=lime ";
    nodes << "]";
	nodes << ";" << std::endl;
}

void DotGraphGenerator::doHandleEdges(Group* node){
	assert (node !=0);

	/* Check that a node is not processed twice e.g. by multiple traversals */
	vector<Node*>::iterator visitedNodes;
	visitedNodes = find (alreadyVisitedNodes.begin(), alreadyVisitedNodes.end(), node);
	if (visitedNodes != alreadyVisitedNodes.end()) { // if in list, skip processing of edge
		return;
	}

	// An egde has the following format: 3 -> 4;

	for (unsigned int childIndex = 0; childIndex < node->getNumberOfChildren(); ++childIndex) {
	    if(config.abbreviateIds) {
	    	edges << "\"" << uuidToUnsignedInt(node->getId()) << "\"" << " -> " << "\"" <<uuidToUnsignedInt((*node->getChild(childIndex)).getId()) << "\"" << ";" << std::endl;
	    } else {
	    	edges << "\"" << node->getId() << "\"" << " -> " << "\"" << (*node->getChild(childIndex)).getId() << "\"" << ";" << std::endl;
		}
	}
}

std::string DotGraphGenerator::getDotGraph() {
	std::string result;

	/* This e.g. would be a valid dot file:
	 * digraph {
	 *	1 [label = "ID [1]\n"];
	 *	2 [label = "ID [2]\n(name = point_cloud_1)\n"];
	 *	3 [label = "ID [3]\n"];
	 *	4 [label = "ID [4]\n(name = point_cloud_2)\n"];
	 *	5 [label = "ID [5]\n(name = point_cloud_reduced)\n(filterType = octree)\n"];
	 *	6 [label = "ID [6]\n(name = point_cloud_box_roi)\n"];
	 *	7 [label = "ID [7]\n"];
	 *	9 [label = "ID [9]\n"];
	 *	10 [label = "ID [10]\n"];
	 *	11 [label = "ID [11]\n(name = mesh_1)\n"];
	 *
	 *	1 -> 2;
	 *	1 -> 3;
	 *	1 -> 5;
	 *	1 -> 6;
	 *	1 -> 7;
	 *	1 -> 11;
	 *	3 -> 4;
	 *	7 -> 9;
	 *	9 -> 10;
	 *
	 *	}
	 */

	dotGraph.str(""); //reset
	dotGraph << "digraph {" << std::endl;
	dotGraph << nodes.str();
	dotGraph << std::endl;
	dotGraph << edges.str();
	dotGraph << std::endl;
	dotGraph << "}" << std::endl;
	result = dotGraph.str();
	return result;
}

std::string DotGraphGenerator::stringify(std::string input) {
	std::string stringifiedInput = input;
//	LOG(DEBUG) << "DotGraphGenerator::stringify input = " << input;
	boost::replace_all(stringifiedInput, "\"", "\\\""); // replace all " with \"
//	LOG(DEBUG) << "DotGraphGenerator::stringify stringifiedInput = " << stringifiedInput;
	return stringifiedInput;
}

}

}

/* EOF */
