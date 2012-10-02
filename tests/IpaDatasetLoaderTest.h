/**
 * @file 
 * IpaDatasetLoaderTest.h
 *
 * @date: Dec 18, 2009
 * @author: sblume
 */

#ifndef IPADATASETLOADERTEST_H_
#define IPADATASETLOADERTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "brics_3d/util/IpaDatasetLoader.h"

#include <string>
#include <iostream>

using namespace std;
using namespace brics_3d;

namespace unitTests {

class IpaDatasetLoaderTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( IpaDatasetLoaderTest );
	CPPUNIT_TEST( testConstructor );
	CPPUNIT_TEST( testLoadData );
	CPPUNIT_TEST( testLoadPointCloud );
	CPPUNIT_TEST_SUITE_END();


public:
	void setUp();
	void tearDown();

	void testConstructor();
	void testLoadData();
	void testLoadPointCloud();


};

	string filename;

	IpaDatasetLoader* loader;

	static const double maxTolerance = 0.00001;

}

#endif /* IPADATASETLOADERTEST_H_ */

/* EOF */
