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

using namespace brics_3d;
using namespace brics_3d::rsg;
using brics_3d::Logger;

/*
 * Example for a function block.
 *
 */
class TestBlock : public IFunctionBlock {
public:

	TestBlock(brics_3d::WorldModel* wmHandle) : IFunctionBlock(wmHandle) {
		name = "testblock";
		Logger::setMinLoglevel(Logger::LOGDEBUG);
		LOG(INFO) << name << ": Starting block " << name;
	};

	~TestBlock(){
		LOG(INFO) << name << ": Stopping block " << name;
	};

	bool configure(brics_3d::ParameterSet parameters) {
		LOG(INFO) << name << ": Configuring parameters.";
		return true;
	}

	bool execute(std::vector<Id>& inputDataIds, std::vector<Id>& outputDataIds) {
		LOG(INFO) << name << ": Executing block " << name << " with " << inputDataIds.size() << " ids as input.";
		return true;
	}

	bool execute(std::string inputModel, std::string& outputModel) {
		LOG(INFO) << name << ": Executing block " << name << " with " << inputModel << " as input model. This is not supported.";
		return false;
	}

private:

	string name;

};

/* Mandatory macros. They define the symbols in this shared library. */
FUNCTION_BLOCK_CREATE(TestBlock)
FUNCTION_BLOCK_DESTROY


/* EOF */
