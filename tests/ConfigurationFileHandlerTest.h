/**
 * @file 
 * ConfigurationFileHandlerTest.h
 *
 * @date: Dec 14, 2009
 * @author: sblume
 */

#ifndef CONFIGURATIONFILEHANDLERTEST_H_
#define CONFIGURATIONFILEHANDLERTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "util/ConfigurationFileHandler.h"
//#include "algorithm/registration/IIterativeClosestPoint.h"

using namespace std;
using namespace BRICS_3D;

namespace unitTests {

class ConfigurationFileHandlerTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( ConfigurationFileHandlerTest );
	CPPUNIT_TEST( testConstructor );
#ifdef BRICS_XERCES_ENABLE //This test can only be performed if xerces is enabled
	CPPUNIT_TEST( testParsing );
#endif
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testConstructor();

#ifdef BRICS_XERCES_ENABLE //This test can only be performed if xerces is enabled
	void testParsing();
#endif


private:

	ConfigurationFileHandler* xmlHandler;

	string filename;

	static const double maxTolerance = 0.00001;
};

}

#endif /* CONFIGURATIONFILEHANDLERTEST_H_ */

/* EOF */
