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
#include <brics_3d/worldModel/sceneGraph/JSONDeserializer.h>

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
	brics_3d::WorldModel* wm = new brics_3d::WorldModel();

	brics_3d::rsg::JSONDeserializer deserializer(wm);
	deserializer.write(serializedModel.str());

	return 0;
}
