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

#include <stdio.h>
#include <fstream>
#include <sstream>

#include <brics_3d/core/Logger.h>
#include <brics_3d/util/JSONTypecaster.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/UuidGenerator.h>
#include <brics_3d/worldModel/sceneGraph/JSONDeserializer.h>
#include <brics_3d/worldModel/sceneGraph/JSONSerializer.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/JSONQueryRunner.h>

using brics_3d::Logger;
using namespace brics_3d;


int main(int argc, char **argv) {
	LOG(INFO) << " JSON parser test.";

	if ((argc != 2) && (argc != 3)) {
		printf("Usage: %s input_file [input_query_file]\n", *argv);
		return 1;
	}

	const char *fileName = argv[1];
	std::ifstream inputFile;
	inputFile.open (fileName, std::ifstream::in);
	std::stringstream serializedModel;
	serializedModel << inputFile.rdbuf();

	std::stringstream serializedQuery;
	if (argc == 3) {
		const char *queryFileName = argv[2];
		std::ifstream queryInputFile;
		queryInputFile.open (queryFileName, std::ifstream::in);
		serializedQuery << queryInputFile.rdbuf();
	}

	/* Create a world model handle */
	Logger::setMinLoglevel(Logger::LOGDEBUG);
	brics_3d::rsg::Id rootId; // Explicitly set the root Id (optinal)
	/*
	 * The rootId is the one ond only information from the model we
	 * to obtain at creation time.
	 */
	rootId = brics_3d::rsg::JSONDeserializer::getRootIdFromJSONModel(serializedModel.str());
	//rootId.fromString("77fd4353-033b-4c23-a675-602fa0e59804");
	brics_3d::WorldModel* wm = new brics_3d::WorldModel(new brics_3d::rsg::UuidGenerator(rootId));

	/* Attach debug visualization */
	brics_3d::rsg::DotVisualizer* graphVizualizer = new brics_3d::rsg::DotVisualizer(&wm->scene); // NOTE: the constructor	                                                                // needs the world model handle.
	brics_3d::rsg::VisualizationConfiguration dotConfiguration; // Again, _optional_ configuration file.
	dotConfiguration.visualizeAttributes = true; // Vizualize attributes of a node iff true.
	dotConfiguration.visualizeIds = true;        // Vizualize Ids of a node iff true.
	dotConfiguration.abbreviateIds = false;       // Vizualize only the lower 2 bytes of anId iff true.
	graphVizualizer->setConfig(dotConfiguration);
	graphVizualizer->setKeepHistory(true); // Do not overwrite the produced files. Default is false.
	graphVizualizer->setFileName("json_graph");
	wm->scene.attachUpdateObserver(graphVizualizer); // Enable graph visualization

	brics_3d::rsg::OSGVisualizer* geometryVizualizer = new brics_3d::rsg::OSGVisualizer(); // Create the visualizer.
	brics_3d::rsg::VisualizationConfiguration osgConfiguration; // _Optional_ configuration file.
	osgConfiguration.visualizeAttributes = true; // Vizualize attributes of a node iff true.
	osgConfiguration.visualizeIds = true;        // Vizualize Ids of a node iff true.
	osgConfiguration.abbreviateIds = true;       // Vizualize only the lower 2 bytes of an Id iff true.
	geometryVizualizer->setConfig(osgConfiguration);
	wm->scene.attachUpdateObserver(geometryVizualizer); // Enable 3D visualization
	wm->scene.advertiseRootNode();						// Inform freshly attached opserver about _this_ world model

	/* Add a JSON serializer (optional) */
	brics_3d::rsg::JSONSerializer serializer(wm, NULL);
	wm->scene.attachUpdateObserver(&serializer); // Enable JSON based serialization
	wm->scene.advertiseRootNode();				 // Inform freshly attached opserver about _this_ world model

	/* Start with a prior set attribute  for the rootIS (optional - but JSONModel should only append) */
	std::vector<brics_3d::rsg::Attribute> attributes;
	wm->scene.getNodeAttributes(rootId, attributes); 				// Get potentially existing attributes
	attributes.push_back(brics_3d::rsg::Attribute("name", "uav_view"));	// Append new attribute
	wm->scene.setNodeAttributes(rootId, attributes);				// Update node

	/* Do the actual JSON parsing. */
	LOG(INFO) << serializedModel.str();
	brics_3d::rsg::JSONDeserializer deserializer(wm);
	deserializer.setMapUnknownParentIdsToRootId(true);
	deserializer.write(serializedModel.str());

	/* (Optionally) apply a query */
	LOG(INFO) << std::endl <<"--------------------QUERY-----------------------" << std::endl;
	LOG(INFO) << serializedQuery.str();
	brics_3d::rsg::JSONQueryRunner queryRunner(wm);
	string query = serializedQuery.str();
	string reply;
	queryRunner.query(query, reply);
	LOG(INFO) << "Reply = " << std::endl << reply;

	/* GUI */
	while(!geometryVizualizer->done()) { // Wait until user closes the GUI-
		// Nothing to do here.
	}

	return 0;
}
