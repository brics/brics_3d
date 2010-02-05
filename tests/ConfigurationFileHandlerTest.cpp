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
	delete xmlHandler;
}

void ConfigurationFileHandlerTest::testConstructor() {

	CPPUNIT_ASSERT(xmlHandler == 0);
	xmlHandler = new ConfigurationFileHandler(filename);
	CPPUNIT_ASSERT(xmlHandler != 0);

//	delete xmlHandler;
}

void ConfigurationFileHandlerTest::testParsing() {
	xmlHandler = new ConfigurationFileHandler(filename);
	CPPUNIT_ASSERT(xmlHandler->getErrorsOccured() == false);

	int maxIterations;
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "maxIterations", &maxIterations) == true);
	CPPUNIT_ASSERT_EQUAL(25, maxIterations);
//	cout << "maxIterations = " << maxIterations << endl;

	double convergenceThreshold;
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "convergenceThreshold", &convergenceThreshold) == true);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.001, convergenceThreshold, maxTolerance);
//	cout << "convergenceThreshold = " << convergenceThreshold << endl;

	string implementation;
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "implementation", &implementation) == true);
	CPPUNIT_ASSERT(implementation.compare("IterativeClosestPoint") == 0);
//	cout << "implementation = " << implementation << endl;

	string assigner;
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("IterativeClosestPoint", "PointCorrespondence", &assigner) == true);
	CPPUNIT_ASSERT(assigner.compare("PointCorrespondenceKDTree") == 0);
//	cout << "assigner = " << assigner << endl;

	string rigidEstimator;
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("IterativeClosestPoint", "RigidTransformationEstimation", &rigidEstimator) == true);
	CPPUNIT_ASSERT(rigidEstimator.compare("RigidTransformationEstimationSVD") == 0);
//	cout << "assigner = " << rigidEstimator << endl;

}

void ConfigurationFileHandlerTest::testFalseParameters() {
	int intDummy;
	double doubleDummy;
	string stringDummy;

	/*
	 * wrong file name
	 */
	xmlHandler = new ConfigurationFileHandler("WrongFileName");
	CPPUNIT_ASSERT(xmlHandler->getErrorsOccured() == true);

	/* algorithm is correct, parameters are correct, but errors occured during parsing */
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "maxIterations", &intDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "convergenceThreshold", &doubleDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "PointCorrespondence", &stringDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("IterativeClosestPoint", "RigidTransformationEstimation", &stringDummy) == false);

	/* algorithm is correct, parameters are wrong*/
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "wrongAttibute", &intDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "wrongAttibute", &doubleDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "wrongAttibute", &stringDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("IterativeClosestPoint", "wrongSubAlgorithm", &stringDummy) == false);

	/* algorithm is wrong, parameters are correct */
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "maxIterations", &intDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "convergenceThreshold", &doubleDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "PointCorrespondence", &stringDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("wrongAlgorithm", "RigidTransformationEstimation", &stringDummy) == false);

	/* algorithm is wrong, parameters are wrong */
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "wrongAttibute", &intDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "wrongAttibute", &doubleDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "wrongAttibute", &stringDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("wrongAlgorithm", "wrongSubAlgorithm", &stringDummy) == false);

	/*
	 * correct file name
	 */
	xmlHandler = new ConfigurationFileHandler(filename);
	CPPUNIT_ASSERT(xmlHandler->getErrorsOccured() == false);

	/* algorithm is correct, parameters are wrong*/
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "wrongAttibute", &intDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "wrongAttibute", &doubleDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("IterativeClosestPoint", "wrongAttibute", &stringDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("IterativeClosestPoint", "wrongSubAlgorithm", &stringDummy) == false);

	/* algorithm is wrong, parameters are correct */
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "maxIterations", &intDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "convergenceThreshold", &doubleDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "PointCorrespondence", &stringDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("wrongAlgorithm", "RigidTransformationEstimation", &stringDummy) == false);

	/* algorithm is wrong, parameters are wrong */
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "wrongAttibute", &intDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "wrongAttibute", &doubleDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getAttribute("wrongAlgorithm", "wrongAttibute", &stringDummy) == false);
	CPPUNIT_ASSERT(xmlHandler->getSubAlgorithm("wrongAlgorithm", "wrongSubAlgorithm", &stringDummy) == false);

}


} // namespace unitTests


/* EOF */
