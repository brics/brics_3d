/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2016, KU Leuven
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
#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>
#include <brics_3d/worldModel/sceneGraph/FunctionBlockLoader.h>
#include <brics_3d/core/Logger.h>

#include <dlfcn.h>

using namespace brics_3d;
using namespace brics_3d::rsg;
using brics_3d::Logger;

/**
 * Example to test if a block can be loaded, executed and unloaded.
 */
int main(int argc, char **argv) {
	Logger::setMinLoglevel(Logger::LOGDEBUG);
	WorldModel wm;

	if(argc <= 2) {
		LOG(ERROR) << "Please use ./loader <path> <blockName>";
		return -1;
	}
	string blockName = argv[2];
	string blockPath = argv[1];

	/* load the block */
	LOG(INFO) << "Loading.";
	FunctionBlockLoader loader;
	FunctionBlockModuleInfo blockInfo;
	if(!loader.loadFunctionBlock(blockName, blockPath, &wm, blockInfo)) {
		LOG(ERROR) << "Cannot load block "<< blockName;
		return -1;
	}
	IFunctionBlock* functionBlock1 = FunctionBlockLoader::moduleToBlock(blockInfo);

	/* execute */
	std::vector<Id> ids;
	functionBlock1->execute(ids, ids);
	functionBlock1->execute(ids, ids);
	Id id1 = 1;
	Id id2 = 2;
	Id id3 = 3;
	ids.push_back(id1);
	ids.push_back(id2);
	ids.push_back(id3);
	functionBlock1->execute(ids, ids);

	string inputModel = "{}";
	string outputModel = "";
	functionBlock1->execute(inputModel,  outputModel);

	/* unload */
	loader.unloadFunctionBlock(blockInfo);
}

/* EOF */
