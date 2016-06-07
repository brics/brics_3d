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

#ifndef RSG_FUNCTIONBLOCKMODULEINFO_H_
#define RSG_FUNCTIONBLOCKMODULEINFO_H_

namespace brics_3d {
namespace rsg {

struct FunctionBlockModuleInfo {
	std::string name; 				// Name as used during loading.
	void* functionBlock; 			// Pointer to the actual block with loose coupling. This is necessary to avoid cyclic dependencies.
									// FunctionBlockModuleInfo actually defines the scpoe/meta-data for this pointer.
	void* moduleHandle;  			// Pointer to the shared library.
};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_FUNCTIONBLOCKMODULEINFO_H_ */

/* EOF */
