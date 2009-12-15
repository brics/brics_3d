/**
 * @file 
 * ConfigurationFileHandlerTest.cpp
 *
 * @date: Dec 14, 2009
 * @author: sblume
 */

#include "ConfigurationFileHandlerTest.h"

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( ConfigurationFileHandlerTest );

void ConfigurationFileHandlerTest::setUp() {

	char defaultFilename[255] = { BRICS_CONFIGURATIONS_DIR};
	strcat(defaultFilename, "/unitTestConfig.xml\0");
	filename = defaultFilename;

}

void ConfigurationFileHandlerTest::tearDown() {

}

void ConfigurationFileHandlerTest::testConstructor() {

	CPPUNIT_ASSERT(xmlHandler == 0);
	xmlHandler = new ConfigurationFileHandler(filename);
	CPPUNIT_ASSERT(xmlHandler != 0);

//	delete xmlHandler;
}

void ConfigurationFileHandlerTest::testParsing() {
	xmlHandler = new ConfigurationFileHandler(filename);

	int maxIterations;
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "maxIterations", &maxIterations) == true);
	CPPUNIT_ASSERT_EQUAL(20, maxIterations);
	cout << "maxIterations = " << maxIterations << endl;

	double convergenceThreshold;
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "convergenceThreshold", &convergenceThreshold) == true);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.001, convergenceThreshold, maxTolerance);
	cout << "convergenceThreshold = " << convergenceThreshold << endl;

	string implementation;
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "implementation", &implementation) == true);
	CPPUNIT_ASSERT(implementation.compare("IterativeClosestPoint") == 0);
	cout << "implementation = " << implementation << endl;

	string assigner;
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("IterativeClosestPoint", "PointCorrespondence", &assigner) == true);
	CPPUNIT_ASSERT(assigner.compare("PointCorrespondenceKDTree") == 0);
	cout << "assigner = " << assigner << endl;

	string rigidEstimator;
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("IterativeClosestPoint", "RigidTransformationEstimation", &rigidEstimator) == true);
	CPPUNIT_ASSERT(rigidEstimator.compare("RigidTransformationEstimationSVD") == 0);
	cout << "assigner = " << rigidEstimator << endl;


}

} // namespace unitTests


/* EOF */
