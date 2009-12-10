/**
 * @file 
 * HomogeneousMatrixTest.cpp
 *
 * @date: Nov 26, 2009
 * @author: sblume
 */

#include "HomogeneousMatrixTest.h"

#include <sstream>

namespace unitTests {
// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( HomogeneousMatrixTest );

void HomogeneousMatrixTest::setUp() {

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

} // namespace unitTests

/* EOF */
