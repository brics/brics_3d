/**
 * @file 
 * FunctionBlockTest.h
 *
 * @date: Dec 18, 2013
 * @author: sblume
 */

#ifndef FUNCTIONBLOCKTEST_H_
#define FUNCTIONBLOCKTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/worldModel/WorldModel.h"

namespace unitTests {



class FunctionBlockTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( FunctionBlockTest );
#ifdef BRICS_MICROBLX_ENABLE
	CPPUNIT_TEST( testFunctionBlockLoader );
	CPPUNIT_TEST( testFunctionBlockExecution );
#endif
	CPPUNIT_TEST_SUITE_END();

public:

	void setUp();
	void tearDown();

	void testFunctionBlockLoader();
	void testFunctionBlockExecution();

private:

	brics_3d::WorldModel* wm;
	std::string functionBlockFile;

};

}  // namespace unitTests
#endif /* FUNCTIONBLOCKTEST_H_ */

/* EOF */
