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
	wm = new brics_3d::WorldModel();
	functionBlockFile = "/home/sblume/brics_sandbox/microblx/microblx/std_blocks/cppdemo/cppdemo.so";
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
	CPPUNIT_FAIL("TODO");
}

}  // namespace unitTests

/* EOF */
