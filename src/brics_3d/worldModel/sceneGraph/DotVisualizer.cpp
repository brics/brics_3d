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

#include "DotVisualizer.h"
#include "brics_3d/core/Logger.h"

using brics_3d::Logger;

namespace brics_3d {

namespace rsg {

DotVisualizer::DotVisualizer(brics_3d::rsg::SceneGraphFacade* scene) : scene(scene)  {
	keepHistory = false;
	counter = 0;
	fileName = "current_graph";
}

DotVisualizer::~DotVisualizer() {

}


bool DotVisualizer::addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId) {
	printGraph();
	return true;
}

bool DotVisualizer::addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId) {
	printGraph();
	return true;
}

bool DotVisualizer::addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, brics_3d::rsg::TimeStamp timeStamp, bool forcedId) {
	printGraph();
	return true;
}

bool DotVisualizer::addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forcedId) {
	printGraph();
	return true;
}


bool DotVisualizer::addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId) {
	printGraph();
	return true;
}

bool DotVisualizer::addRemoteRootNode(Id rootId, vector<Attribute> attributes) {
	printGraph();
	return true;
}

bool DotVisualizer::addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId) {
	printGraph();
	return true;
}



bool DotVisualizer::setNodeAttributes(Id id, vector<Attribute> newAttributes, TimeStamp timeStamp) {
	printGraph();
	return true;
}

bool DotVisualizer::setTransform(Id id, brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,  brics_3d::rsg::TimeStamp timeStamp) {
	printGraph();
	return true;
}

bool DotVisualizer::setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp) {
	printGraph();
	return true;
}


bool DotVisualizer::deleteNode(Id id) {
	printGraph();
	return true;
}

bool DotVisualizer::addParent(Id id, Id parentId) {
	printGraph();
	return true;
}

bool DotVisualizer::removeParent(Id id, Id parentId) {
	printGraph();
	return true;
}


void DotVisualizer::printGraph(){
	LOG(DEBUG) << "DotVisualizer: Printing graph to file.";
	graphPrinter.setConfig(config);
	scene->executeGraphTraverser(&graphPrinter, scene->getRootId());

	/* Save a svg file as snapshopt */
	// e.g.  "current_graph.gv";
	output.open((fileName + ".gv").c_str(), std::ios::trunc);
	if (!output.fail()) {
		output << graphPrinter.getDotGraph();
	} else {
		LOG(ERROR) << "DotVisualizer: Cannot write to file " << fileName << ".gv";
	}

	output.flush();
	output.close();

	std::stringstream command;
	command.str("");
	// e.g. "dot current_graph.gv -Tsvg -o current_graph.gv.svg";
	command << "dot " << fileName <<".gv -Tsvg -o " << fileName << ".gv.svg";
	system(command.str().c_str()); //e.g. with gthumb you can observe changes...

	if(keepHistory) {
		command.str("");

		command << "cp " << fileName <<".gv.svg "<< fileName << "_graph_" << counter << ".gv.svg";
		system(command.str().c_str());
		command.str("");
		command << "cp " << fileName <<".gv "<< fileName << "_graph_" << counter << ".gv";
		system(command.str().c_str());
		LOG(DEBUG) << "DotVisualizer:	File name is: "<< fileName << "_graph_" << counter << ".gv.svg";
	}

	graphPrinter.reset();
	counter++;
}


};

}

/* EOF */
