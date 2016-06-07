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

#ifndef RSG_FUNCTIONBLOCKLOADER_H_
#define RSG_FUNCTIONBLOCKLOADER_H_

#include <brics_3d/worldModel/sceneGraph/IFunctionBlock.h>
#include <brics_3d/worldModel/sceneGraph/FunctionBlockModuleInfo.h>
#include <brics_3d/core/Logger.h>

#include <dlfcn.h>

namespace brics_3d {

namespace rsg {

//struct FunctionBlockModuleInfo {
//	std::string name; 				// Name as used during loading.
////	IFunctionBlock* functionBlock; 	// Pointer to the actual block.
//	void* functionBlock; 			// Pointer to the actual block.
//	void* moduleHandle;  			// Pointer to the shared library.
//};

/// Definitions for class factories that are part of the plugin.
typedef IFunctionBlock* create_t(brics_3d::WorldModel* wmHandle);
typedef void destroy_t(IFunctionBlock*);

/**
 * @brief Loads function blocks as plugins (shared libraries).
 *
 * Based on dlopen().
 */
class FunctionBlockLoader {
public:
	FunctionBlockLoader(std::string defaultPath);
	FunctionBlockLoader();
	virtual ~FunctionBlockLoader();

    bool loadFunctionBlock(std::string name,
    		brics_3d::WorldModel* wmHandle,
    		brics_3d::rsg::FunctionBlockModuleInfo& module); //default path

    bool loadFunctionBlock(std::string name,
    		std::string path,
    		brics_3d::WorldModel* wmHandle,
    		brics_3d::rsg::FunctionBlockModuleInfo& module);

    bool unloadFunctionBlock(brics_3d::rsg::FunctionBlockModuleInfo& module);

    static IFunctionBlock* moduleToBlock(brics_3d::rsg::FunctionBlockModuleInfo& module);

private:

	std::string defaultPath;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_FUNCTIONBLOCKLOADER_H_ */

/* EOF */
