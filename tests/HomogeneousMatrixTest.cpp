/**
 * @file 
 * HomogeneousMatrixTest.cpp
 *
 * @date: Nov 26, 2009
 * @author: sblume
 */

#include "HomogeneousMatrixTest.h"

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

//	cout << endl;
//	for (int i = 0; i < arraySize; ++i) {
//		cout << matrixPtr[i];
//		if (((i + 1) % 4) == 0) {
//			cout << endl;
//		}
//	}
//	cout << endl;


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

} // namespace unitTests

/* EOF */
