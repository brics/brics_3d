/**
 * @file 
 * HomogeneousMatrixTest.cpp
 *
 * @date: Nov 26, 2009
 * @author: sblume
 */

#include "HomogeneousMatrixTest.h"

using namespace brics_3d::matrixEntry;

#include <sstream>

namespace unitTests {
// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( HomogeneousMatrixTest );

void HomogeneousMatrixTest::setUp() {
	matrix = 0;
}

void HomogeneousMatrixTest::tearDown() {

}

void HomogeneousMatrixTest::testDefaultConstructor() {

	matrix = new HomogeneousMatrix44();
	matrixPtr = matrix->getRawData();

	/* rotation in column-major order*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

	delete matrix;
}

void HomogeneousMatrixTest::testOveloadedConstructor() {

	matrix = new HomogeneousMatrix44(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 1.0, 2.0, 3.0);
	matrixPtr = matrix->getRawData();

	/* rotation in column-major order */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(9.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

	delete matrix;
}

void HomogeneousMatrixTest::testEigenConstructor() {

	/* create Identity matrix with Eigen */
	Transform3d transformation;
	transformation = Transform3d::Identity();

	matrix = new HomogeneousMatrix44(&transformation);
	matrixPtr = matrix->getRawData();

	/* rotation in column-major order */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

	delete matrix;

}

void HomogeneousMatrixTest::testPolymorphConstructor() {

	/*
	 * Test default constructor
	 */
	IHomogeneousMatrix44* abstractMatrix = new HomogeneousMatrix44();
	matrixPtr = abstractMatrix->getRawData();

	/* rotation in column-major order*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);
	delete abstractMatrix;

	/*
	 * Test overloaded constructor
	 */
	abstractMatrix = new HomogeneousMatrix44(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 1.0, 2.0, 3.0);
	matrixPtr = abstractMatrix->getRawData();

	/* rotation in column-major order */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(9.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);
	delete abstractMatrix;

	/*
	 * Test with Eigen matrix
	 */

	/* create Identity matrix with Eigen */
	Transform3d transformation;
	transformation = Transform3d::Identity();

	abstractMatrix = new HomogeneousMatrix44(&transformation);
	matrixPtr = abstractMatrix->getRawData();

	/* rotation in column-major order */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);
	delete abstractMatrix;

}

void HomogeneousMatrixTest::testStreaming() {
	string comparatorString;
	string referenceString = "1 0 0 0\n0 1 0 0\n0 0 1 0\n0 0 0 1\n";
	stringstream testStringStream0;

	matrix = new HomogeneousMatrix44();

	/* test output */
	testStringStream0 << *matrix;
	comparatorString = testStringStream0.str();
	CPPUNIT_ASSERT_EQUAL(0, comparatorString.compare(referenceString));

	delete matrix;

}

void HomogeneousMatrixTest::testAssignment() {
	IHomogeneousMatrix44* matrix1 = new HomogeneousMatrix44(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 1.0, 2.0, 3.0);
	IHomogeneousMatrix44* matrix2 = new HomogeneousMatrix44();

	matrixPtr = matrix2->getRawData();
	/* rotation in column-major order */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

	/* perform assignment operator */
	*matrix2 = *matrix1;

	matrixPtr = matrix2->getRawData();
	/* rotation in column-major order */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(9.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);


	delete matrix1;
	delete matrix2;
}

void HomogeneousMatrixTest::testMultiplication() {
	IHomogeneousMatrix44* matrix1 = new HomogeneousMatrix44(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 1.0, 2.0, 3.0);
	IHomogeneousMatrix44* matrix2 = new HomogeneousMatrix44();
	IHomogeneousMatrix44* result = new HomogeneousMatrix44();

//	cout << *result;
	*result = *((*matrix2) * (*matrix1));
//	cout << *result;

	matrixPtr = result->getRawData();

	/* rotation in column-major order */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[4], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[8], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, matrixPtr[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, matrixPtr[5], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, matrixPtr[9], maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, matrixPtr[2], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, matrixPtr[6], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(9.0, matrixPtr[10], maxTolerance);

	/* translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, matrixPtr[14], maxTolerance);

	/* stuffing coefficients */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[3], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[7], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[11], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, matrixPtr[15], maxTolerance);

	delete matrix1;
	delete matrix2;
	delete result;
}

void HomogeneousMatrixTest::testInverse() {
	HomogeneousMatrix44 pureTranslation(1,0,0, 0,1,0, 0,0,1, 1,2,3);

//	cout << "pureTranslation:" << endl << pureTranslation << endl;
	pureTranslation.inverse();
//	cout << "inverse of pureTranslation:" << endl << pureTranslation << endl;
	matrixPtr = pureTranslation.getRawData();

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, matrixPtr[14], maxTolerance);

	HomogeneousMatrix44 pureTranslation2(1,0,0, 0,1,0, 0,0,1, 3,4,5);
	HomogeneousMatrix44 pureTranslation3(1,0,0, 0,1,0, 0,0,1, 3,4,5);
	pureTranslation3.inverse();

	IHomogeneousMatrix44* result = new HomogeneousMatrix44();
	result = pureTranslation2 * pureTranslation3;
	CPPUNIT_ASSERT(result->isIdentity() == true);


	/* now translation and rotation */

	AngleAxis<double> rotation(M_PI_2/4.0, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	transformation.translate(Vector3d(5,6,99.9));
	IHomogeneousMatrix44* someTransform = new HomogeneousMatrix44(&transformation);
	IHomogeneousMatrix44* someOtherTransform = new HomogeneousMatrix44();
	*someOtherTransform = *someTransform;
	someTransform->inverse();
//	cout << "someTransform:" << endl << *someTransform << endl;

	result = (*someTransform) * (*someOtherTransform);
//	cout << "result:" << endl << *result << endl;
	CPPUNIT_ASSERT(result->isIdentity() == true);


}

void HomogeneousMatrixTest::testIsIdentity() {
	HomogeneousMatrix44 identity;
	HomogeneousMatrix44 noIdentity(1,0,0, 0,1,0, 0,0,1, 1,2,3);
	HomogeneousMatrix44 nearlyIdentity(1.001,0,0, 0,1,0, 0,0,0.999, 0,0,0.001);

	CPPUNIT_ASSERT(identity.isIdentity() == true);
	CPPUNIT_ASSERT(noIdentity.isIdentity() == false);
	CPPUNIT_ASSERT(nearlyIdentity.isIdentity() == false);
	CPPUNIT_ASSERT(nearlyIdentity.isIdentity(0.01) == true); // check with less precision
}

void HomogeneousMatrixTest::testRPYConversions() {
	double xExpected, yExpected, zExpected, rollExpected, pitchExpected, yawExpected;
	double xActual, yActual, zActual, rollActual, pitchActual, yawActual;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform1(new HomogeneousMatrix44());

	xExpected = 1.0;
	yExpected = 2.0;
	zExpected = 3.0;
	rollExpected = 0.5*M_PI;
	pitchExpected = 0.0;
	yawExpected = 0.0;

	HomogeneousMatrix44::xyzRollPitchYawToMatrix(xExpected, yExpected, zExpected, rollExpected, pitchExpected, yawExpected, transform1);
	std::cout << "---> transform1 " << std::endl <<*transform1;
	HomogeneousMatrix44::matrixToXyzRollPitchYaw(transform1, xActual, yActual, zActual, rollActual, pitchActual, yawActual);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(xExpected, xActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(yExpected, yActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(zExpected, zActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(rollExpected, rollActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(pitchExpected, pitchActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(yawExpected, yawActual, maxTolerance);

	xExpected = 1.0;
	yExpected = 2.0;
	zExpected = 3.0;
	rollExpected = 0.0;
	pitchExpected = 0.0;
	yawExpected = 0.5 * M_PI;

	Eigen::AngleAxis<double> rotation2(yawExpected, Eigen::Vector3d(0,0,1));
	Transform3d transformation2;
	transformation2 = Eigen::Affine3d::Identity();
	transformation2.translate(Eigen::Vector3d(xExpected,yExpected,zExpected));
	transformation2.rotate(rotation2);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform2(new HomogeneousMatrix44(&transformation2));


	HomogeneousMatrix44::matrixToXyzRollPitchYaw(transform2, xActual, yActual, zActual, rollActual, pitchActual, yawActual);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(xExpected, xActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(yExpected, yActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(zExpected, zActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(rollExpected, rollActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(pitchExpected, pitchActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(yawExpected, yawActual, maxTolerance);

	xExpected = 1.0;
	yExpected = 2.0;
	zExpected = 3.0;
	rollExpected = 0.0;
	pitchExpected = 0.5 * M_PI;
	yawExpected = 0;

	Eigen::AngleAxis<double> rotation3(pitchExpected, Eigen::Vector3d(0,1,0));
	Transform3d transformation3;
	transformation3 = Eigen::Affine3d::Identity();
	transformation3.translate(Eigen::Vector3d(xExpected,yExpected,zExpected));
	transformation3.rotate(rotation3);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform3(new HomogeneousMatrix44(&transformation3));

	HomogeneousMatrix44::matrixToXyzRollPitchYaw(transform3, xActual, yActual, zActual, rollActual, pitchActual, yawActual);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(xExpected, xActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(yExpected, yActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(zExpected, zActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(rollExpected, rollActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(pitchExpected, pitchActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(yawExpected, yawActual, maxTolerance);

	xExpected = 1.0;
	yExpected = 2.0;
	zExpected = 3.0;
	rollExpected = 0.5 * M_PI;
	pitchExpected = 0.0;
	yawExpected = 0;

	Eigen::AngleAxis<double> rotation4(rollExpected, Eigen::Vector3d(1,0,0));
	Transform3d transformation4;
	transformation4 = Eigen::Affine3d::Identity();
	transformation4.translate(Eigen::Vector3d(xExpected,yExpected,zExpected));
	transformation4.rotate(rotation4);
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform4(new HomogeneousMatrix44(&transformation4));


	HomogeneousMatrix44::matrixToXyzRollPitchYaw(transform4, xActual, yActual, zActual, rollActual, pitchActual, yawActual);
	std::cout << "---> transform4 " << std::endl <<*transform4;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(xExpected, xActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(yExpected, yActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(zExpected, zActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(rollExpected, rollActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(pitchExpected, pitchActual, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(yawExpected, yawActual, maxTolerance);

	/* CHecke if Eigen generated and xyzRPY generated _matices_ are similar */
	const double* matrixData1 = transform1->getRawData();
	const double* matrixData4 = transform4->getRawData();
	for (unsigned int i = 0; i < 16; ++i) {
		CPPUNIT_ASSERT_DOUBLES_EQUAL(matrixData1[i], matrixData4[i], maxTolerance);
	}
}

void HomogeneousMatrixTest::testMatrixEntries() {
	CPPUNIT_ASSERT(0 == brics_3d::matrixEntry::r11);
	CPPUNIT_ASSERT(4 == brics_3d::matrixEntry::r12);
	CPPUNIT_ASSERT(8 == brics_3d::matrixEntry::r13);

	CPPUNIT_ASSERT(1 == brics_3d::matrixEntry::r21);
	CPPUNIT_ASSERT(5 == brics_3d::matrixEntry::r22);
	CPPUNIT_ASSERT(9 == brics_3d::matrixEntry::r23);

	CPPUNIT_ASSERT(2 == brics_3d::matrixEntry::r31);
	CPPUNIT_ASSERT(6 == brics_3d::matrixEntry::r32);
	CPPUNIT_ASSERT(10 == brics_3d::matrixEntry::r33);

	CPPUNIT_ASSERT(12 == brics_3d::matrixEntry::x);
	CPPUNIT_ASSERT(13 == brics_3d::matrixEntry::y);
	CPPUNIT_ASSERT(14 == brics_3d::matrixEntry::z);

	// check namespace clashes
	CPPUNIT_ASSERT(0 == r11);
	CPPUNIT_ASSERT(4 == r12);
	CPPUNIT_ASSERT(8 == r13);

	CPPUNIT_ASSERT(1 == r21);
	CPPUNIT_ASSERT(5 == r22);
	CPPUNIT_ASSERT(9 == r23);

	CPPUNIT_ASSERT(2 == r31);
	CPPUNIT_ASSERT(6 == r32);
	CPPUNIT_ASSERT(10 == r33);

	CPPUNIT_ASSERT(12 == x);
	CPPUNIT_ASSERT(13 == y);
	CPPUNIT_ASSERT(14 == z);
}

void HomogeneousMatrixTest::testDistance() {
	double distance = -1.0;

	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform000(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0,0,0)); 						// Translation coefficients
	HomogeneousMatrix44::matrixToDistance(transform000, distance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, distance, maxTolerance);


	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform111(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,1,1)); 						// Translation coefficients

	HomogeneousMatrix44::matrixToDistance(transform111, distance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.732050808, distance, maxTolerance);

	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform222(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             2,2,2)); 						// Translation coefficients

	HomogeneousMatrix44::matrixToDistance(transform222, distance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.4641, distance, maxTolerance);
}

} // namespace unitTests

/* EOF */
