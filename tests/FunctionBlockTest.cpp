/**
 * @file 
 * FunctionBlockTest.cpp
 *
 * @date: Dec 18, 2013
 * @author: sblume
 */

#include "FunctionBlockTest.h"
#include "brics_3d/core/Logger.h"

namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( FunctionBlockTest );

void FunctionBlockTest::setUp() {
#ifdef BRICS_MICROBLX_ENABLE
	CPPUNIT_ASSERT(brics_3d::WorldModel::microBlxWmHandle == 0);
#endif
	wm = new brics_3d::WorldModel();
#ifdef BRICS_MICROBLX_ENABLE
	CPPUNIT_ASSERT(brics_3d::WorldModel::microBlxWmHandle == wm);
#endif
//	functionBlockFile = "/home/sblume/brics_sandbox/microblx/microblx/std_blocks/cppdemo/cppdemo.so";
	functionBlockFile = "cppdemo";

	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
}

void FunctionBlockTest::tearDown() {
	delete wm;
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::WARNING);
}

void FunctionBlockTest::testFunctionBlockLoader() {
	CPPUNIT_ASSERT(!wm->loadFunctionBlock("wrongFileName"));
	CPPUNIT_ASSERT(wm->loadFunctionBlock(functionBlockFile));
}

void FunctionBlockTest::testFunctionBlockExecution() {
	CPPUNIT_ASSERT(wm->loadFunctionBlock(functionBlockFile));

	vector<brics_3d::rsg::Id> input;
	input.push_back(wm->getRootNodeId()); // input hook
	input.push_back(wm->getRootNodeId()); // output hook
	vector<brics_3d::rsg::Id> output;
	CPPUNIT_ASSERT(!wm->executeFunctionBlock("wrongFileName", input, output));
	CPPUNIT_ASSERT(wm->executeFunctionBlock(functionBlockFile, input, output));
	CPPUNIT_ASSERT(wm->executeFunctionBlock(functionBlockFile, input, output));
	CPPUNIT_ASSERT(wm->executeFunctionBlock(functionBlockFile, input, output));
}

}  // namespace unitTests

/* EOF */
