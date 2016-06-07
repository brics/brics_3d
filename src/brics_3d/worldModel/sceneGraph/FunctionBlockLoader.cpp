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

#include "FunctionBlockLoader.h"

namespace brics_3d {
namespace rsg {

FunctionBlockLoader::FunctionBlockLoader() {
	this->defaultPath = ".";
}

FunctionBlockLoader::FunctionBlockLoader(std::string defaultPath) {
	this->defaultPath = defaultPath;
}

FunctionBlockLoader::~FunctionBlockLoader() {

}

bool FunctionBlockLoader::loadFunctionBlock(std::string name,
		brics_3d::WorldModel* wmHandle,
		brics_3d::rsg::FunctionBlockModuleInfo& module) {

	return loadFunctionBlock(name, defaultPath, wmHandle, module);
}

bool FunctionBlockLoader::loadFunctionBlock(std::string name,
		std::string path,
		brics_3d::WorldModel* wmHandle,
		brics_3d::rsg::FunctionBlockModuleInfo& module) {

	/* Initialize result with empty values in case of an error */
	module.name = name;
	module.functionBlock = 0;
	module.moduleHandle = 0;

	/* load the library */
	string blockName = path + "/" + name + ".so";
	void* blockHandle = dlopen(blockName.c_str(), RTLD_LAZY);
	if (!blockHandle) {
		LOG(ERROR) << "FunctionBlockLoader: Cannot load library: " << dlerror() << '\n';
		return false;
	}

	/* Reset errors */
	dlerror();

	/* Load the symbols */
	create_t* create_block = (create_t*) dlsym(blockHandle, "create");
	const char* dlsym_error = dlerror();
	if (dlsym_error) {
		LOG(ERROR) << "FunctionBlockLoader: Cannot load symbol create: " << dlsym_error << '\n';
		return false;
	}

	destroy_t* destroy_block = (destroy_t*) dlsym(blockHandle, "destroy");
	dlsym_error = dlerror();
	if (dlsym_error) {
		LOG(ERROR) << "FunctionBlockLoader: Cannot load symbol destroy: " << dlsym_error << '\n';
		return false;
	}

	/* Create an instance of the class */
	IFunctionBlock* functionBlock = create_block(wmHandle);

	/* Set result */
	module.functionBlock = functionBlock;
	module.moduleHandle = blockHandle;

	LOG(DEBUG) << "FunctionBlockLoader: block " << module.name << " successfully loaded.";
	return true;
}

bool FunctionBlockLoader::unloadFunctionBlock(brics_3d::rsg::FunctionBlockModuleInfo& module) {

	if ((module.functionBlock) == 0 || (module.moduleHandle == 0)) {
		LOG(ERROR) << "FunctionBlockLoader::unloadFunctionBlock invalid module meta-data for block " << module.name;
		return false;
	}

	destroy_t* destroy_block = (destroy_t*) dlsym(module.moduleHandle, "destroy");
	const char* dlsym_error = dlerror();
	if (dlsym_error) {
		LOG(ERROR) << "FunctionBlockLoader: Cannot load symbol destroy: " << dlsym_error << '\n';
		return false;
	}

	// destroy the class
	destroy_block(module.functionBlock);
	module.functionBlock = 0;

	// unload
	dlclose(module.moduleHandle);
	module.moduleHandle = 0;

	LOG(DEBUG) << "FunctionBlockLoader: block " << module.name << " successfully unloaded.";
	return true;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
