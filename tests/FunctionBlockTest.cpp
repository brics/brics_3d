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
//	functionBlockFile = "/home/sblume/sandbox/microblx/microblx/std_blocks/cppdemo/cppdemo.so";
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

void FunctionBlockTest::testExternalFunctionBlockExecution() {
//	string blockName = "cppdemo";
//	string blockPath = "/home/sblume/sandbox/microblx/microblx/std_blocks/cppdemo/";
	string blockName = "roifilter";//"testblock";
	string blockPath = "/home/sblume/sandbox/brics_3d_function_blocks/lib/";

	CPPUNIT_ASSERT(wm->loadFunctionBlock(blockName, blockPath));

	vector<brics_3d::rsg::Id> input;
	brics_3d::rsg::Id inputHook = 1042; //wm->getRootNodeId();
	input.push_back(inputHook); // input hook
	input.push_back(wm->getRootNodeId()); // output hook
	vector<brics_3d::rsg::Id> output;
	CPPUNIT_ASSERT(wm->executeFunctionBlock(blockName, input, output));

	/* modify mw */
	brics_3d::rsg::Id groupId = 0;
	std::vector<brics_3d::rsg::Attribute> attributes;
	attributes.push_back(brics_3d::rsg::Attribute("name","test_group1"));
	wm->scene.addGroup(wm->getRootNodeId(), groupId, attributes);

	CPPUNIT_ASSERT(wm->executeFunctionBlock(blockName, input, output));
//	CPPUNIT_ASSERT(wm->loadFunctionBlock(blockName, blockPath));  //not yet working
	CPPUNIT_ASSERT(wm->executeFunctionBlock(blockName, input, output));
}

}  // namespace unitTests

/* EOF */
