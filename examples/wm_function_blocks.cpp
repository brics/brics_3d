/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2014, KU Leuven
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

/* BRICS_3D includes */

#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/core/PointCloud3D.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>

using brics_3d::Logger;

int main(int argc, char **argv) {

	std::string pathToFunctionBlocksRepository;
	std::string dataSetFolder = BRICS_MODELS_DIR;
	std::string dataSet = dataSetFolder + "/bunny000.txt";

	/* process input */
	if (argc != 2) {
		LOG(ERROR) << "Usage: ./" << argv[0] << " <path_to_function_blocks_root_folder>";
		return -1;
	} else {
		pathToFunctionBlocksRepository = argv[1];
	}

	/* Configure logger - default level won't tell us much */
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	/* Create WM  handle */
	brics_3d::WorldModel* wm = new brics_3d::WorldModel();

	/* Attach some visualization facilities */
	brics_3d::rsg::OSGVisualizer* wm3DVisualizer = new brics_3d::rsg::OSGVisualizer();
	brics_3d::rsg::DotVisualizer* wmStructureVisualizer = new brics_3d::rsg::DotVisualizer(&wm->scene);
	brics_3d::rsg::VisualizationConfiguration osgConfiguration; // optional configuration
	osgConfiguration.visualizeIds = true;
	osgConfiguration.visualizeAttributes = true;
	osgConfiguration.visualizeTransforms = false;
	wm3DVisualizer->setConfig(osgConfiguration);
	wm->scene.attachUpdateObserver(wm3DVisualizer); //enable visualization
//	wm->scene.attachUpdateObserver(wmStructureVisualizer);
	wm->scene.advertiseRootNode(); // Don't forget this one! Otherwise the observers cannot correctly handle the updates.

	/* Load a data set */
	brics_3d::PointCloud3D::PointCloud3DPtr pointCloud1(new brics_3d::PointCloud3D());
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pointCloud1Container(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	pointCloud1Container->data = pointCloud1;
	pointCloud1Container->data->readFromTxtFile(dataSet);

	/* Add point cloud to wm */
    brics_3d::rsg::Id pointCloud1Id = 0;
    std::vector<brics_3d::rsg::Attribute> attributes;
    attributes.clear();
    attributes.push_back(brics_3d::rsg::Attribute("name","pointCloud1Id"));
	//wm->scene.addGeometricNode(wm->getRootNodeId(), pointCloud1Id, attributes, pointCloud1Container, wm->now());


	/*
	 * Load function blocks
	 */
	//string blockName = "roifilter";
	string blockPath = pathToFunctionBlocksRepository + "/lib/";
	wm->loadFunctionBlock("pointcloudloader", blockPath);
	wm->loadFunctionBlock("octreefilter", blockPath);
	wm->loadFunctionBlock("osmloader", blockPath);

	/*
	 * Execute function blocks
	 */
	vector<brics_3d::rsg::Id> input;
	brics_3d::rsg::Id inputHook = pointCloud1Id;
	brics_3d::rsg::Id outputHook = wm->getRootNodeId();
	input.push_back(outputHook); // output input hook
	input.push_back(inputHook); // input hook
	vector<brics_3d::rsg::Id> output;
	vector<brics_3d::rsg::Id> output2;
//	wm->executeFunctionBlock("pointcloudloader", input, output);
	wm->executeFunctionBlock("osmloader", input, output);
	wm->executeFunctionBlock("octreefilter", output, output2); // "stack them together"

	/* Wait until user closes the GUI */
	while(!wm3DVisualizer->done()) {
		//nothing here
	}

	/* Clean up */
	delete wm3DVisualizer;
	delete wmStructureVisualizer;
	delete wm;
}

/* EOF */
