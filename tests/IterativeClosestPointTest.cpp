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
#include <stdexcept>

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

void IterativeClosestPointTest::testSimpleAlignmentSVD() {
	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
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
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getX(), (*pointCloudCubeCopy->getPointCloud())[i].getX(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getY(), (*pointCloudCubeCopy->getPointCloud())[i].getY(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getZ(), (*pointCloudCubeCopy->getPointCloud())[i].getZ(), maxTolerance);
	}


	delete homogeneousTrans;
	delete resultTransformation;
}

void IterativeClosestPointTest::testSimpleAlignmentQUAT() {
	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);

	/* set up ICP */
	IPointCorrespondence* assigner;
	assigner = new PointCorrespondenceKDTree();
	IRigidTransformationEstimation* estimator = new RigidTransformationEstimationQUAT();
	icp = new IterativeClosestPoint(assigner, estimator);

	/* perform ICP*/
	IHomogeneousMatrix44* resultTransformation = new HomogeneousMatrix44();
	icp->match(pointCloudCube, pointCloudCubeCopy, resultTransformation);

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = resultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix1[i]), abs(matrix2[i]), maxTolerance);
	}

	/* test if aligned point cloud is the more or less same as the initial one */
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getX(), (*pointCloudCubeCopy->getPointCloud())[i].getX(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getY(), (*pointCloudCubeCopy->getPointCloud())[i].getY(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getZ(), (*pointCloudCubeCopy->getPointCloud())[i].getZ(), maxTolerance);
	}

	delete homogeneousTrans;
	delete resultTransformation;
}

void IterativeClosestPointTest::testSimpleAlignmentHELIX() {
	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);

	/* set up ICP */
	IPointCorrespondence* assigner;
	assigner = new PointCorrespondenceKDTree();
	IRigidTransformationEstimation* estimator = new RigidTransformationEstimationHELIX();
	icp = new IterativeClosestPoint(assigner, estimator);

	/* perform ICP*/
	IHomogeneousMatrix44* resultTransformation = new HomogeneousMatrix44();
	icp->match(pointCloudCube, pointCloudCubeCopy, resultTransformation);

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = resultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix1[i]), abs(matrix2[i]), maxTolerance * 10e3); //HELIX is not very accurate...
	}

	/* test if aligned point cloud is the more or less same as the initial one */
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getX(), (*pointCloudCubeCopy->getPointCloud())[i].getX(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getY(), (*pointCloudCubeCopy->getPointCloud())[i].getY(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getZ(), (*pointCloudCubeCopy->getPointCloud())[i].getZ(), maxTolerance);
	}

	delete homogeneousTrans;
	delete resultTransformation;
}

void IterativeClosestPointTest::testSimpleAlignmentAPX() {
	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);

	/* set up ICP */
	IPointCorrespondence* assigner;
	assigner = new PointCorrespondenceKDTree();
	IRigidTransformationEstimation* estimator = new RigidTransformationEstimationAPX();
	icp = new IterativeClosestPoint(assigner, estimator);

	/* perform ICP*/
	IHomogeneousMatrix44* resultTransformation = new HomogeneousMatrix44();
	icp->match(pointCloudCube, pointCloudCubeCopy, resultTransformation);

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = resultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix1[i]), abs(matrix2[i]), maxTolerance);
	}

	/* test if aligned point cloud is the more or less same as the initial one */
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getX(), (*pointCloudCubeCopy->getPointCloud())[i].getX(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getY(), (*pointCloudCubeCopy->getPointCloud())[i].getY(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((int)(*pointCloudCube->getPointCloud())[i].getZ(), (*pointCloudCubeCopy->getPointCloud())[i].getZ(), maxTolerance);
	}

	delete homogeneousTrans;
	delete resultTransformation;
}

void IterativeClosestPointTest::testStatefullInterface() {
	/* manipulate second point cloud */
	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	IHomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(&transformation);
	pointCloudCubeCopy->homogeneousTransformation(homogeneousTrans);
	PointCloud3D* tmpPointCloudCube;
	PointCloud3D* tmpPointCloudCubeCopy;

	/* set up ICP */
	IPointCorrespondence* assigner;
	assigner = new PointCorrespondenceKDTree();
	IRigidTransformationEstimation* estimator = new RigidTransformationEstimationSVD();
	icp = new IterativeClosestPoint(assigner, estimator);


	IHomogeneousMatrix44* resultTransformation;// = new HomogeneousMatrix44();
	IHomogeneousMatrix44* tmpResultTransformation = new HomogeneousMatrix44();
	icp->setModel(pointCloudCube);
	icp->setData(pointCloudCubeCopy);
	tmpPointCloudCube = icp->getModel();
	tmpPointCloudCubeCopy = icp->getData();
	for (unsigned int i = 0; i < pointCloudCube->getSize(); ++i) { // "model" and "data" should not have changed...
		CPPUNIT_ASSERT_DOUBLES_EQUAL((*pointCloudCube->getPointCloud())[i].getX(), (*tmpPointCloudCube->getPointCloud())[i].getX(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((*pointCloudCube->getPointCloud())[i].getY(), (*tmpPointCloudCube->getPointCloud())[i].getY(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((*pointCloudCube->getPointCloud())[i].getZ(), (*tmpPointCloudCube->getPointCloud())[i].getZ(), maxTolerance);

		CPPUNIT_ASSERT_DOUBLES_EQUAL((*pointCloudCubeCopy->getPointCloud())[i].getX(), (*tmpPointCloudCubeCopy->getPointCloud())[i].getX(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((*pointCloudCubeCopy->getPointCloud())[i].getY(), (*tmpPointCloudCubeCopy->getPointCloud())[i].getY(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((*pointCloudCubeCopy->getPointCloud())[i].getZ(), (*tmpPointCloudCubeCopy->getPointCloud())[i].getZ(), maxTolerance);
	}

	/* perform ICP */
	for (int i = 0; i < 3; ++i) { //here we know empirically that 3 iterations are enough
			icp->performNextIteration();
			*tmpResultTransformation = *((*icp->getLastEstimatedTransformation()) * (*tmpResultTransformation));
	}

	resultTransformation = icp->getAccumulatedTransfomation();
	CPPUNIT_ASSERT(resultTransformation != 0);

	/* test if initial and resulting homogeneous transformations are the same */
	const double* matrix1 = homogeneousTrans->getRawData();
	const double* matrix2 = resultTransformation->getRawData();
	const double* matrix3 = tmpResultTransformation->getRawData();

	for (int i = 0; i < 16; ++i) {
//		cout << matrix1[i] << ", " << matrix2[i] << endl; //DBG output
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix1[i]), abs(matrix2[i]), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL(abs(matrix2[i]), abs(matrix3[i]), maxTolerance);
//		CPPUNIT_ASSERT_DOUBLES_EQUAL(matrix1[i], matrix2[i], maxTolerance);
	}

	/* test if aligned point cloud is the more or less same as the initial one */
	for (unsigned int i = 0;  i < pointCloudCube->getSize(); ++ i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL((*pointCloudCube->getPointCloud())[i].getX(), (*pointCloudCubeCopy->getPointCloud())[i].getX(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((*pointCloudCube->getPointCloud())[i].getY(), (*pointCloudCubeCopy->getPointCloud())[i].getY(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL((*pointCloudCube->getPointCloud())[i].getZ(), (*pointCloudCubeCopy->getPointCloud())[i].getZ(), maxTolerance);
	}

	delete tmpResultTransformation;
	delete homogeneousTrans;
}

void IterativeClosestPointTest::testSetupInterface(){
	IPointCorrespondence* assigner0 = 0;
	IPointCorrespondence* assigner1 = 0;
	IPointCorrespondence* assigner2 = 0;
	assigner0 = new PointCorrespondenceKDTree();
	IRigidTransformationEstimation* estimator0 = 0;
	IRigidTransformationEstimation* estimator1 = 0;
	IRigidTransformationEstimation* estimator2 = 0;
	estimator0 = new RigidTransformationEstimationSVD();
	IIterativeClosestPointSetup* icpSetup = 0;

	icp = new IterativeClosestPoint(assigner0, estimator0);
	CPPUNIT_ASSERT(icpSetup == 0);
	icpSetup = dynamic_cast<IIterativeClosestPointSetup*>(icp);
	CPPUNIT_ASSERT(icpSetup != 0);

	/* check subalgorithms, set within constructor */
	CPPUNIT_ASSERT(assigner0 != assigner1);
	assigner1 = icpSetup->getAssigner();
	CPPUNIT_ASSERT(assigner0 == assigner1);

	CPPUNIT_ASSERT(estimator0 != estimator1);
	estimator1 = icpSetup->getEstimator();
	CPPUNIT_ASSERT(estimator0 == estimator1);

	/* check subalgorithms, set within setup interface */
	delete assigner0;
	delete estimator0;
	assigner1 = new PointCorrespondenceKDTree();
	estimator1 = new RigidTransformationEstimationSVD();
	icp->setAssigner(assigner1);
	icp->setEstimator(estimator1);

	CPPUNIT_ASSERT(assigner1 != assigner2);
	assigner2 = icpSetup->getAssigner();
	CPPUNIT_ASSERT(assigner1 == assigner2);

	CPPUNIT_ASSERT(estimator1 != estimator2);
	estimator2 = icpSetup->getEstimator();
	CPPUNIT_ASSERT(estimator1 == estimator2);

	/* check parameters in setup interface */
	int maxIterations = 123;
	icpSetup->setMaxIterations(maxIterations);
	CPPUNIT_ASSERT_EQUAL(maxIterations, icpSetup->getMaxIterations());

	maxIterations = 456;
	icpSetup->setMaxIterations(maxIterations);
	CPPUNIT_ASSERT_EQUAL(maxIterations, icpSetup->getMaxIterations());

	maxIterations = -1;
	CPPUNIT_ASSERT_THROW(icpSetup->setMaxIterations(maxIterations), runtime_error);

	double convergenceThreshold = 0.01;
	icpSetup->setConvergenceThreshold(convergenceThreshold);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(convergenceThreshold, icpSetup->getConvergenceThreshold(), maxTolerance);

	convergenceThreshold = 456;
	icpSetup->setConvergenceThreshold(convergenceThreshold);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(convergenceThreshold, icpSetup->getConvergenceThreshold(), maxTolerance);

	convergenceThreshold = -1.0;
	CPPUNIT_ASSERT_THROW(icpSetup->setConvergenceThreshold(convergenceThreshold), runtime_error);


}

}  // namespace unitTests

/* EOF */
