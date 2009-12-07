/**
 * @file 
 * RigidTransformationEstimationTest.cpp
 *
 * @date: Dec 7, 2009
 * @author: sblume
 */

#include "RigidTransformationEstimationTest.h"

#include <sstream>

using namespace Eigen;

CPPUNIT_TEST_SUITE_REGISTRATION( RigidTransformationEstimationTest );

void RigidTransformationEstimationTest::setUp() {
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
	delete pointCloudCube;
	delete pointCloudCubeCopy;

	delete estimator;
	delete abstractEstimator;
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
	AngleAxis<double> rotation(M_PI_2l/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);

	/* we already know the correspondences...*/
	vector<CorrespondencePoint3DPair>* pointPairs = new vector<CorrespondencePoint3DPair>();
	for (int i = 0; i < pointCloudCube->getSize(); ++i) {
		Point3D firstPoint = (*pointCloudCube->getPointCloud())[i];
		Point3D secondPoint = (*pointCloudCubeCopy->getPointCloud())[i];

		CorrespondencePoint3DPair tmpPair(firstPoint, secondPoint);
		pointPairs->push_back(tmpPair);
	}

	/* perform estimation */
	IHomogeneousMatrix44* reusultTransformation = new HomogeneousMatrix44();
	estimator = new RigidTransformationEstimationSVD();
	estimator->estimateTransformation(pointCloudCube, pointCloudCubeCopy, pointPairs, reusultTransformation);

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = reusultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) { //TODO getSize() for IHomogeneousMatrix44 ?!?
		//cout << matrix1[i] << ", " << matrix2[i] << endl;
		CPPUNIT_ASSERT_DOUBLES_EQUAL(matrix1[i], matrix2[i], maxTolerance);
	}

//	CPPUNIT_ASSERT(false);
}

/* EOF */
