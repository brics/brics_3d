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

	void configure(brics_3d::ParameterSet parameters) {
		LOG(INFO) << name << ": Configuring parameters.";
	}


	void execute() {
		LOG(INFO) << name << ": Executing block " << name << " with " << inputDataIds.size() << " ids as input.";

	}

private:

	string name;

};

extern "C" IFunctionBlock* create(brics_3d::WorldModel* wmHandle) {
    return new TestBlock(wmHandle);
}

extern "C" void destroy(IFunctionBlock* block) {
    delete block;
}

/* EOF */
