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
#include <brics_3d/core/Logger.h>

#include <dlfcn.h>

using namespace brics_3d;
using namespace brics_3d::rsg;
using brics_3d::Logger;

// the types of the class factories
typedef IFunctionBlock* create_t(brics_3d::WorldModel* wmHandle);
typedef void destroy_t(IFunctionBlock*);

int main(int argc, char **argv) {
	Logger::setMinLoglevel(Logger::LOGDEBUG);
	WorldModel wm;

	if(argc <= 1) {
		LOG(ERROR) << argc << "Please use ./loader <blockName.so>";
		return -1;
	}
	string blockName = argv[1];
	LOG(INFO) << "Loading.";


	// load the triangle library
	void* blockHandle = dlopen(blockName.c_str(), RTLD_LAZY);
	if (!blockHandle) {
		LOG(ERROR) << "Cannot load library: " << dlerror() << '\n';
		return 1;
	}

	// reset errors
	dlerror();

	// load the symbols
	create_t* create_block = (create_t*) dlsym(blockHandle, "create");
	const char* dlsym_error = dlerror();
	if (dlsym_error) {
		LOG(ERROR) << "Cannot load symbol create: " << dlsym_error << '\n';
		return 1;
	}

	destroy_t* destroy_block = (destroy_t*) dlsym(blockHandle, "destroy");
	dlsym_error = dlerror();
	if (dlsym_error) {
		LOG(ERROR) << "Cannot load symbol destroy: " << dlsym_error << '\n';
		return 1;
	}

	// create an instance of the class
	IFunctionBlock* functionBlock1 = create_block(&wm);

	// do something
	functionBlock1->execute();
	functionBlock1->execute();
	std::vector<Id> ids;
	Id id1 = 1;
	Id id2 = 2;
	Id id3 = 3;
	ids.push_back(id1);
	ids.push_back(id2);
	ids.push_back(id3);
	functionBlock1->setData(ids);
	functionBlock1->execute();

	// destroy the class
	destroy_block(functionBlock1);

	// unload
	dlclose(blockHandle);

}

/* EOF */
