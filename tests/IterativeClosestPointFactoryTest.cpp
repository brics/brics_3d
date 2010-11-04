/**
 * @file 
 * IterativeClosestPointFactoryTest.cpp
 *
 * @date: Dec 15, 2009
 * @author: sblume
 */

#include "IterativeClosestPointFactoryTest.h"
#include "algorithm/registration/IterativeClosestPoint.h"
#include "algorithm/registration/PointCorrespondenceKDTree.h"
#include "algorithm/registration/RigidTransformationEstimationSVD.h"

#include <cstring>
#include <typeinfo>

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( IterativeClosestPointFactoryTest );

void IterativeClosestPointFactoryTest::setUp() {
	icpFactory = 0;
	char defaultFilename[255] = { BRICS_CONFIGURATIONS_DIR};
	strcat(defaultFilename, "/unitTestConfig.xml\0");
	filename = defaultFilename;
}

void IterativeClosestPointFactoryTest::tearDown() {
	if (icpFactory) {
		delete icpFactory;
		icpFactory = 0;
	}
}

void IterativeClosestPointFactoryTest::testConstructor() {
	CPPUNIT_ASSERT(icpFactory == 0);
	icpFactory = new IterativeClosestPointFactory();
	CPPUNIT_ASSERT(icpFactory != 0);

}

void IterativeClosestPointFactoryTest::testDefault() {
	icpFactory = new IterativeClosestPointFactory();
	IIterativeClosestPointPtr icp;
	icp = icpFactory->createIterativeClosestPoint();
	CPPUNIT_ASSERT(icp != 0);
}

void IterativeClosestPointFactoryTest::testUnitTestConfig() {
	icpFactory = new IterativeClosestPointFactory();
	IIterativeClosestPointPtr icp;
	IIterativeClosestPointSetupPtr icpConfigurator;
	icp = icpFactory->createIterativeClosestPoint(filename);
	icpConfigurator = icpFactory->getIcpSetupHandle();
	CPPUNIT_ASSERT(icp != 0);

	CPPUNIT_ASSERT_EQUAL(25, icpConfigurator->getMaxIterations());
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.001, icpConfigurator->getConvergenceThreshold(), maxTolerance);

	//IterativeClosestPoint* genericIcpImpl;
	boost::shared_ptr<IterativeClosestPoint> genericIcpImpl;
//	genericIcpImpl = dynamic_cast<IterativeClosestPoint*>(icp); //polymorph downcast
	genericIcpImpl = boost::shared_dynamic_cast<IterativeClosestPoint>(icp); //polymorph downcast

	IPointCorrespondence* assigner = genericIcpImpl->getAssigner();
	PointCorrespondenceKDTree assignerReference;
//    cout << "*assigner is: " << typeid(*assigner).name() << endl; //DBG output
//    cout << "assignerReference is: " << typeid(assignerReference).name() << endl;
    CPPUNIT_ASSERT((typeid(*assigner) == typeid(assignerReference)));

	IRigidTransformationEstimation* estimator = genericIcpImpl->getEstimator();
	RigidTransformationEstimationSVD estimatorReference;
//    cout << "*estimator is: " << typeid(*estimator).name() << endl; //DBG output
//    cout << "estimatorReference is: " << typeid(estimatorReference).name() << endl;
    CPPUNIT_ASSERT((typeid(*estimator) == typeid(estimatorReference)));

}

}

/* EOF */
