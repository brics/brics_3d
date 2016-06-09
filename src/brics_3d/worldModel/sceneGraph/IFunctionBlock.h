/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
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

#ifndef RSG_IFUNCTIONBLOCK_H_
#define RSG_IFUNCTIONBLOCK_H_

#include <vector>
#include <brics_3d/core/ParameterSet.h>
#include <brics_3d/worldModel/WorldModel.h>

namespace brics_3d {

namespace rsg {

/* Macros that define the symbols that are searched on the shared libraries  */
#define FUNCTION_BLOCK_CREATE(BlockImplementation) \
extern "C" __attribute__ ((visibility("default"))) brics_3d::rsg::IFunctionBlock* create(brics_3d::WorldModel* wmHandle) {return new BlockImplementation(wmHandle);}

#define FUNCTION_BLOCK_DESTROY \
extern "C" __attribute__ ((visibility("default"))) void destroy(brics_3d::rsg::IFunctionBlock* block) {delete block;}

/**
 * A function block as it will be used for perception algorithms or advanced queries within the world model.
 * It is loaded as shared library and can be executed at runtime.
 */
class IFunctionBlock {
public:

	IFunctionBlock(brics_3d::WorldModel* wmHandle){
		this->wm = wmHandle;
	};

	virtual ~IFunctionBlock(){};

	/**
	 * @brief Configure a function block with a set of parameters.
	 * @param parameters Parameters as key-value list.
	 * @return True on success.
	 */
	virtual bool configure(brics_3d::ParameterSet parameters) = 0;

	/**
	 * @brief Execute a function block based sets of ids.
	 *
	 * The id sets follow the convention the first id being an "outputhook".
	 * I.e. in case new data has to be store, this will be the parent node.
	 * Note, this interface offers a rather loose coupling of data types. A
	 * more explicit coupling is achieved via the other execute() function
	 * below.
	 *
	 * @param inputDataIds Set of ids to define he input. First id must be the output hook.
	 * @param outputDataIds Set of ids to define he input.
	 * @return True on success.
	 */
	virtual bool execute(std::vector<Id>& inputDataIds, std::vector<Id>& outputDataIds) = 0;

	/**
	 * @brief Execute a function block based on model based inputs an outputs in JSON encoding.
	 * @param inputModel A JSON model to define the input. Must contain a "metadata" field referring
	 *                   to a JSON Schema that validated the input. The Schema must be retrievable by
	 *                   the respective getMetaModel() method.
	 * @param outputModel Like inputModel, but for the output.
	 * @return True on success.
	 */
	virtual bool execute(std::string inputModel, std::string& outputModel) = 0;

	/**
	 * @brief Get the meta models of a function block.
	 * @param inputMetaModel Meta model encoded as JSON Schema that defines a valid input JSON model.
	 *                       An empty meta model "{}" indicates the function block does not support
	 *                       model based i/o.
	 * @param outputMetaModel Like inputMetaModel, but for the output.
	 * @return True on success. False in case model based i/o is not supported.
	 */
	virtual bool getMetaModel(std::string& inputMetaModel, std::string& outputMetaModel) = 0;


protected:

	/// Handle to the world model that stores all 3D data.
	brics_3d::WorldModel* wm;

private:
	IFunctionBlock(){};

};

}

}

#endif /* RSG_IFUNCTIONBLOCK_H_ */

/* EOF */
