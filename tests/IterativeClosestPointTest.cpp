/**
 * @file 
 * RigidTransformationEstimationTest.cpp
 *
 * @date: Dec 7, 2009
 * @author: sblume
 */

#include "IterativeClosestPointTest.h"

#include <sstream>
#include <cmath>

using namespace Eigen;

namespace unitTests {


CPPUNIT_TEST_SUITE_REGISTRATION( IterativeClosestPointTest );

void IterativeClosestPointTest::setUp() {
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

void IterativeClosestPointTest::tearDown() {
	delete pointCloudCube;
	delete pointCloudCubeCopy;

	delete icp;
	delete abstractIcp;
}

void IterativeClosestPointTest::testConstructor() {

	CPPUNIT_ASSERT(icp == 0);
	icp = new IterativeClosestPoint();
	CPPUNIT_ASSERT(icp != 0);

	/* polymorph variant */
	CPPUNIT_ASSERT(abstractIcp == 0);
	abstractIcp = new IterativeClosestPoint();
	CPPUNIT_ASSERT(abstractIcp != 0);

}

void IterativeClosestPointTest::testSimpleAlignment() {
	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2l/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);

	/* set up ICP */
	IPointCorrespondence* assigner;
	assigner = new PointCorrespondenceKDTree();
	IRigidTransformationEstimation* estimator = new RigidTransformationEstimationSVD();
	icp = new IterativeClosestPoint(assigner, estimator);

	/* perform ICP*/
	IHomogeneousMatrix44* resultTransformation = new HomogeneousMatrix44();
	icp->match(pointCloudCube, pointCloudCubeCopy, resultTransformation);

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = resultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) {
//		cout << matrix1[i] << ", " << matrix2[i] << endl; //DBG output
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix1[i]), abs(matrix2[i]), maxTolerance);
//		CPPUNIT_ASSERT_DOUBLES_EQUAL(matrix1[i], matrix2[i], maxTolerance);
	}

	/* test if aligned point cloud is the more or less same as the initial one */
	for (int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].x, (*pointCloudCubeCopy->getPointCloud())[i].x, maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].y, (*pointCloudCubeCopy->getPointCloud())[i].y, maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].z, (*pointCloudCubeCopy->getPointCloud())[i].z, maxTolerance);
	}

}

}  // namespace unitTests

/* EOF */
