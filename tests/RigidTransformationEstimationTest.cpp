/**
 * @file 
 * RigidTransformationEstimationTest.cpp
 *
 * @date: Dec 7, 2009
 * @author: sblume
 */

#include "RigidTransformationEstimationTest.h"
#include "6dslam/src/newmat/newmat.h"

#include <sstream>
#include <cmath>
#include <exception>

using namespace Eigen;

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( RigidTransformationEstimationTest );

void RigidTransformationEstimationTest::setUp() {
	estimator = 0;
	abstractEstimator = 0;
	pointCloudCube = new PointCloud3D();

	point000 = new Point3D(0,0,0);
	point001 = new Point3D(0,0,1);
	point011 = new Point3D(0,1,1);
	point010 = new Point3D(0,1,0);
	point100 = new Point3D(1,0,0);
	point101 = new Point3D(1,0,1);
	point111 = new Point3D(1,1,1);
	point110 = new Point3D(1,1,0);

	pointCloudCube->addPoint(point000);
	pointCloudCube->addPoint(point001);
	pointCloudCube->addPoint(point011);
	pointCloudCube->addPoint(point010);
	pointCloudCube->addPoint(point100);
	pointCloudCube->addPoint(point101);
	pointCloudCube->addPoint(point111);
	pointCloudCube->addPoint(point110);

	pointCloudCubeCopy = new PointCloud3D();
	stringstream tmpSteam;
	tmpSteam << *pointCloudCube;
	tmpSteam >> *pointCloudCubeCopy;
}

void RigidTransformationEstimationTest::tearDown() {
	delete point000;
	delete point001;
	delete point011;
	delete point010;
	delete point100;
	delete point101;
	delete point111;
	delete point110;

	delete pointCloudCube;
	delete pointCloudCubeCopy;

	if (estimator) {
		delete estimator;
		estimator = 0;
	}

	if (abstractEstimator) {
		delete abstractEstimator;
		abstractEstimator = 0;
	}

}

void RigidTransformationEstimationTest::testConstructor() {

	CPPUNIT_ASSERT(estimator == 0);
	estimator = new RigidTransformationEstimationSVD();
	CPPUNIT_ASSERT(estimator != 0);

	/* polymorph variant */
	CPPUNIT_ASSERT(abstractEstimator == 0);
	abstractEstimator = new RigidTransformationEstimationSVD();
	CPPUNIT_ASSERT(abstractEstimator != 0);

}

void RigidTransformationEstimationTest::testSVDTransformation(){

	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);

	/* we already know the correspondences...*/
	vector<CorrespondencePoint3DPair>* pointPairs = new vector<CorrespondencePoint3DPair>();
	for (unsigned int i = 0; i < pointCloudCube->getSize(); ++i) {
		Point3D firstPoint = (*pointCloudCube->getPointCloud())[i];
		Point3D secondPoint = (*pointCloudCubeCopy->getPointCloud())[i];

		CorrespondencePoint3DPair tmpPair(firstPoint, secondPoint);
		pointPairs->push_back(tmpPair);
	}

	/* perform estimation */
	double errorResult1 = 0.0;
	IHomogeneousMatrix44* reusultTransformation = new HomogeneousMatrix44();
	estimator = new RigidTransformationEstimationSVD();
	errorResult1 = estimator->estimateTransformation(pointPairs, reusultTransformation);
	LOG(DEBUG) << "SVD RMS point-to-point error is " << errorResult1;

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = reusultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) { //TODO getSize() for IHomogeneousMatrix44 ?!?
//		cout << matrix1[i] << ", " << matrix2[i] << endl; //DBG output
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix1[i]), abs(matrix2[i]), maxTolerance);
	}

	/* perform estimation again, should be indempotent */
	double errorResult2 = 0.0;
	errorResult2 = estimator->estimateTransformation(pointPairs, reusultTransformation);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(errorResult1, errorResult2, maxTolerance);

	delete reusultTransformation;
	delete pointPairs;
	delete homogeneousTrans;
}

void RigidTransformationEstimationTest::testQUATTransformation() {
	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);

	/* we already know the correspondences...*/
	vector<CorrespondencePoint3DPair>* pointPairs = new vector<CorrespondencePoint3DPair>();
	for (unsigned int i = 0; i < pointCloudCube->getSize(); ++i) {
		Point3D firstPoint = (*pointCloudCube->getPointCloud())[i];
		Point3D secondPoint = (*pointCloudCubeCopy->getPointCloud())[i];

		CorrespondencePoint3DPair tmpPair(firstPoint, secondPoint);
		pointPairs->push_back(tmpPair);
	}

	/* perform estimation */
	double errorResult1 = 0.0;
	IHomogeneousMatrix44* reusultTransformation = new HomogeneousMatrix44();
	abstractEstimator = new RigidTransformationEstimationQUAT();
	errorResult1 = abstractEstimator->estimateTransformation(pointPairs, reusultTransformation);
	LOG(DEBUG) << "QUAT RMS point-to-point error is " << errorResult1;

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = reusultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix1[i]), abs(matrix2[i]), maxTolerance);
	}

	/* perform estimation again, should be indempotent */
	double errorResult2 = 0.0;
	errorResult2 = abstractEstimator->estimateTransformation(pointPairs, reusultTransformation);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(errorResult1, errorResult2, maxTolerance);

	delete reusultTransformation;
	delete pointPairs;
	delete homogeneousTrans;
}

void RigidTransformationEstimationTest::testHELIXTransformation() {
	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);

	/* we already know the correspondences...*/
	vector<CorrespondencePoint3DPair>* pointPairs = new vector<CorrespondencePoint3DPair>();
	for (unsigned int i = 0; i < pointCloudCube->getSize(); ++i) {
		Point3D firstPoint = (*pointCloudCube->getPointCloud())[i];
		Point3D secondPoint = (*pointCloudCubeCopy->getPointCloud())[i];

		CorrespondencePoint3DPair tmpPair(firstPoint, secondPoint);
		pointPairs->push_back(tmpPair);
	}

	/* perform estimation */
	double errorResult1 = 0.0;
	IHomogeneousMatrix44* reusultTransformation = new HomogeneousMatrix44();
	abstractEstimator = new RigidTransformationEstimationHELIX();
	errorResult1 = abstractEstimator->estimateTransformation(pointPairs, reusultTransformation);
	LOG(DEBUG)  << "HELIX RMS point-to-point error is " << errorResult1;

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = reusultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix1[i]), abs(matrix2[i]), maxTolerance * 10e3); //HELIX is not very accurate
	}

	/* perform estimation again, should be indempotent */
	double errorResult2 = 0.0;
	errorResult2 = abstractEstimator->estimateTransformation(pointPairs, reusultTransformation);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(errorResult1, errorResult2, maxTolerance);

	delete reusultTransformation;
	delete pointPairs;
	delete homogeneousTrans;
}

void RigidTransformationEstimationTest::testAPXTransformation() {
	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);

	/* we already know the correspondences...*/
	vector<CorrespondencePoint3DPair>* pointPairs = new vector<CorrespondencePoint3DPair>();
	for (unsigned int i = 0; i < pointCloudCube->getSize(); ++i) {
		Point3D firstPoint = (*pointCloudCube->getPointCloud())[i];
		Point3D secondPoint = (*pointCloudCubeCopy->getPointCloud())[i];

		CorrespondencePoint3DPair tmpPair(firstPoint, secondPoint);
		pointPairs->push_back(tmpPair);
	}

	/* perform estimation */
	double errorResult1 = 0.0;
	IHomogeneousMatrix44* reusultTransformation = new HomogeneousMatrix44();
	abstractEstimator = new RigidTransformationEstimationAPX();
	errorResult1 = abstractEstimator->estimateTransformation(pointPairs, reusultTransformation);
	LOG(DEBUG) << "APX RMS point-to-point error is " << errorResult1;

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = reusultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix1[i]), abs(matrix2[i]), maxTolerance);
	}

	/* perform estimation again, should be indempotent */
	double errorResult2 = 0.0;
	errorResult2 = abstractEstimator->estimateTransformation(pointPairs, reusultTransformation);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(errorResult1, errorResult2, maxTolerance);

	delete reusultTransformation;
	delete pointPairs;
	delete homogeneousTrans;
}

void RigidTransformationEstimationTest::testORTHOTransformation() {
	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);

	/* we already know the correspondences...*/
	vector<CorrespondencePoint3DPair>* pointPairs = new vector<CorrespondencePoint3DPair>();
	for (unsigned int i = 0; i < pointCloudCube->getSize(); ++i) {
		Point3D firstPoint = (*pointCloudCube->getPointCloud())[i];
		Point3D secondPoint = (*pointCloudCubeCopy->getPointCloud())[i];

		CorrespondencePoint3DPair tmpPair(firstPoint, secondPoint);
		pointPairs->push_back(tmpPair);
	}

	/* perform estimation */
	double errorResult1 = 0.0;
	IHomogeneousMatrix44* reusultTransformation = new HomogeneousMatrix44();
	abstractEstimator = new RigidTransformationEstimationORTHO();
	try {
		errorResult1 = abstractEstimator->estimateTransformation(pointPairs, reusultTransformation);
	} catch (exception& e) {
		cerr << "ERROR: RigidTransformationEstimationORTHO has thrown an exception." << endl;
	}
	cout << "INFO: ORTHO RMS point-to-point error is " << errorResult1 << endl;

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = reusultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix1[i]), abs(matrix2[i]), maxTolerance);
	}

	/* perform estimation again, should be indempotent */
	double errorResult2 = 0.0;
	errorResult2 = abstractEstimator->estimateTransformation(pointPairs, reusultTransformation);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(errorResult1, errorResult2, maxTolerance);
}

}  // namespace unitTests

/* EOF */
