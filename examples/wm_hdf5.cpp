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

/*
 * This example illustrates how o read a HDF5 append only log file.
 *
 */

/* BRICS_3D includes */
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/UuidGenerator.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateSerializer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateDeserializer.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>

using namespace brics_3d;
using brics_3d::Logger;


int main(int argc, char **argv) {

	LOG(INFO) << " HDF5 log file parser example.";

	if ((argc != 2) && (argc != 3)) {
		printf("Usage: %s input_file\n", *argv);
		return 1;
	}

	const char *fileName = argv[1];


	/* Create a world model handle */
	Logger::setMinLoglevel(Logger::LOGERROR);
	brics_3d::rsg::Id rootId; // Explicitly set the root Id (optional)
	/*
	 * The rootId is the one and only information from the model we
	 * to obtain at creation time.
	 */
	rootId = brics_3d::rsg::HDF5UpdateDeserializer::getRootIdFromAppendOnlyLogFile(fileName);
	LOG(INFO) << "Root id = " << rootId.toString();
	brics_3d::WorldModel* wm = new brics_3d::WorldModel(new brics_3d::rsg::UuidGenerator(rootId));

	/* Attach debug visualization */
	brics_3d::rsg::DotVisualizer* graphVizualizer = new brics_3d::rsg::DotVisualizer(&wm->scene); // NOTE: the constructor	                                                                // needs the world model handle.
	brics_3d::rsg::VisualizationConfiguration dotConfiguration; // Again, _optional_ configuration file.
	dotConfiguration.visualizeAttributes = true; // Vizualize attributes of a node iff true.
	dotConfiguration.visualizeIds = true;        // Vizualize Ids of a node iff true.
	dotConfiguration.abbreviateIds = false;       // Vizualize only the lower 2 bytes of anId iff true.
	graphVizualizer->setConfig(dotConfiguration);
	graphVizualizer->setKeepHistory(true); // Do not overwrite the produced files. Default is false.
	graphVizualizer->setFileName("hdf5_append_only_graph");
	wm->scene.attachUpdateObserver(graphVizualizer); // Enable graph visualization

#ifdef BRICS_OSG_ENABLE
	brics_3d::rsg::OSGVisualizer* geometryVizualizer = new brics_3d::rsg::OSGVisualizer(); // Create the visualizer.
	brics_3d::rsg::VisualizationConfiguration osgConfiguration; // _Optional_ configuration file.
	osgConfiguration.visualizeAttributes = true; // Vizualize attributes of a node iff true.
	osgConfiguration.visualizeIds = true;        // Vizualize Ids of a node iff true.
	osgConfiguration.abbreviateIds = true;       // Vizualize only the lower 2 bytes of an Id iff true.
	geometryVizualizer->setConfig(osgConfiguration);
	wm->scene.attachUpdateObserver(geometryVizualizer); // Enable 3D visualization
	wm->scene.advertiseRootNode();
#endif

	/* Do the actual HDF5 log file parsing. */
	brics_3d::rsg::HDF5UpdateDeserializer deserializer(wm);
	deserializer.loadFromAppendOnlyLogFile(fileName);


#ifdef BRICS_OSG_ENABLE
	/* Wait until user closes the GUI */
	while(!geometryVizualizer->done()) {
		//nothing here
	}
#endif


	return 0;
}


/* EOF */
