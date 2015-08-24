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
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>

using brics_3d::Logger;
using namespace brics_3d;


int main(int argc, char **argv) {
	LOG(INFO) << " JSON parser test.";

	if (argc != 2) {
		printf("Usage: %s input_file\n", *argv);
		return 1;
	}

	const char *fileName = argv[1];
	std::ifstream inputFile;
	inputFile.open (fileName, std::ifstream::in);
	std::stringstream serializedModel;
	serializedModel << inputFile.rdbuf();

	/* Create a world model handle */
	Logger::setMinLoglevel(Logger::LOGDEBUG);
	brics_3d::rsg::Id rootId; // Explicitly set the root Id (optinal)
	rootId.fromString("77fd4353-033b-4c23-a675-602fa0e59804");
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

	/* Do the actual JSON parsing. */
	LOG(INFO) << serializedModel.str();
	brics_3d::rsg::JSONDeserializer deserializer(wm);
	deserializer.setMapUnknownParentIdsToRootId(true);
	deserializer.write(serializedModel.str());

	return 0;
}
