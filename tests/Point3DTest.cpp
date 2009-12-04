/**
 * @file 
 * Point3DTest.cpp
 *
 * @date: Oct 16, 2009
 * @author: sblume
 */

#include <limits>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "Point3DTest.h"
#include "core/HomogeneousMatrix44.h"


namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( Point3DTest );

void Point3DTest::setUp() {

	/* use absolute max/min of double value */
	maxDouble = std::numeric_limits<double>::max();
	//minDouble = std::numeric_limits<double>::min();
	minDouble = -maxDouble;

	point000 = new Point3D;
	point111 = new Point3D(1, 1, 1);
	pointMinus123 = new Point3D(-1.0, -2.0, -3.0);
	pointMax = new Point3D(maxDouble, maxDouble, maxDouble);
	pointMin = new Point3D(minDouble, minDouble, minDouble);

	referenceVector << 1, 1, 1;
}

void Point3DTest::tearDown() {
	delete point000;
	delete point111;
	delete pointMinus123;
	delete pointMax;
	delete pointMin;
}

void Point3DTest::testConstructor() {

	/*
	 * Test if all instances are created correctly.
	 */
	CPPUNIT_ASSERT((point000->x - 0) < maxTolerance);
	CPPUNIT_ASSERT((point000->x - 0) < maxTolerance);
	CPPUNIT_ASSERT((point000->x - 0) < maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, point000->x, maxTolerance); //same as before but with CppUnit macro
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, point000->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, point000->z, maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->z, maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, pointMinus123->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, pointMinus123->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, pointMinus123->z, maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, pointMax->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, pointMax->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, pointMax->z, maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, pointMin->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, pointMin->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, pointMin->z, maxTolerance);

	/* copy constructor */
	Point3D *newPoint0;
	newPoint0 = new Point3D(point111);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint0->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint0->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint0->z, maxTolerance);
	delete newPoint0;

	Point3D newPoint1(point111);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint1.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint1.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint1.z, maxTolerance);

}

void Point3DTest::testSimleAdditions() {
	Point3D resultPoint(0, 0, 0);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultPoint.z, maxTolerance);

	resultPoint = *point000 + point111;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.z, maxTolerance);

	/* reset result point */
	resultPoint.x = 1.0;
	resultPoint.y = 1.0;
	resultPoint.z = 1.0;

	resultPoint = resultPoint + pointMinus123;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, resultPoint.z, maxTolerance);

	/* reset result point */
	resultPoint.x = 1.0;
	resultPoint.y = 1.0;
	resultPoint.z = 1.0;

}

void Point3DTest::testComplexAdditions() {
	Point3D resultPoint;
	resultPoint = *point000 + pointMax;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, resultPoint.z, maxTolerance);

	CPPUNIT_ASSERT_NO_THROW((resultPoint = *point000 + point000));

	/* check limits */
	CPPUNIT_ASSERT_THROW((resultPoint = *pointMax + pointMax), runtime_error);

}

void Point3DTest::testSimleSubtractions() {
	Point3D resultPoint;

	resultPoint = *point000 - point111;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, resultPoint.z, maxTolerance);

	/* reset result point */
	resultPoint.x = 0.0;
	resultPoint.y = 0.0;
	resultPoint.z = 0.0;

	resultPoint = resultPoint - pointMinus123;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, resultPoint.z, maxTolerance);

}

void Point3DTest::testComplexSubtractions() {
	Point3D resultPoint;
	resultPoint = *point000 - pointMin;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(((-1)*minDouble), resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(((-1)*minDouble), resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(((-1)*minDouble), resultPoint.z, maxTolerance);

	resultPoint = *point000 + pointMin;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, resultPoint.z, maxTolerance);

	CPPUNIT_ASSERT_NO_THROW((resultPoint = *point000 - point000));

	/* check limits */
	CPPUNIT_ASSERT_THROW((resultPoint = *pointMin - pointMax), runtime_error);
	CPPUNIT_ASSERT_THROW((resultPoint = *pointMin + pointMin), runtime_error);

}

void Point3DTest::testSimleMultiplications() {
	Point3D resultPoint;

	resultPoint = *point111 * 10;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0, resultPoint.z, maxTolerance);

	CPPUNIT_ASSERT_NO_THROW((resultPoint = *point111 * 0));
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultPoint.z, maxTolerance);

	resultPoint = *pointMinus123 * (-1);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, resultPoint.z, maxTolerance);
}

void Point3DTest::testComplexMultiplications() {
	Point3D resultPoint;
	resultPoint = (*point111) * (maxDouble);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, resultPoint.z, maxTolerance);

	resultPoint = (*point111) * (minDouble);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, resultPoint.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, resultPoint.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, resultPoint.z, maxTolerance);

	/* check limits */
	CPPUNIT_ASSERT_NO_THROW((resultPoint = (*pointMax) * (1.0) ));
	CPPUNIT_ASSERT_NO_THROW((resultPoint = (*pointMin) * (1.0) ));
	CPPUNIT_ASSERT_THROW((resultPoint = (*pointMax) * (2.0) ), runtime_error);
	CPPUNIT_ASSERT_THROW((resultPoint = (*pointMin) * (2.0) ), runtime_error);

}

void Point3DTest::testStreaming() {
	string comparatorString;
	stringstream testStringStream0;
	stringstream testStringStream1;
	Point3D streamedPoint111;
	Point3D streamedPointMinus123;

	/* test output */
	testStringStream0 << *point111;
	comparatorString = testStringStream0.str();
	CPPUNIT_ASSERT(comparatorString.compare("1 1 1") == 0);

	comparatorString.clear();
	testStringStream1 << *pointMinus123;
	comparatorString = testStringStream1.str();
	CPPUNIT_ASSERT(comparatorString.compare("-1 -2 -3") == 0);

	/* test input */
	testStringStream0 >> streamedPoint111;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, streamedPoint111.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, streamedPoint111.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, streamedPoint111.z, maxTolerance);

	testStringStream1 >> streamedPointMinus123;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, streamedPointMinus123.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, streamedPointMinus123.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, streamedPointMinus123.z, maxTolerance);
}

void Point3DTest::testIdentity() {

	transformation = Transform3d::Identity();

//	//print out
//	double *data;
//	data = transformation.data();
//	for (int i = 0; i < 16; ++i) {
//		cout << data[i] << " ";
//		if ( (i+1)%4 == 0 ) {
//			cout << endl;
//		}
//	}

//	cout << transformation * referenceVector << endl; //expected

	/* check identity */
	HomogeneousMatrix44 *homogeneousTransformation = new HomogeneousMatrix44(&transformation);
	point111->homogeneousTransformation(homogeneousTransformation);
//	cout << *point111 << endl; // actual

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->z, maxTolerance);

	/* check HomogeneousMatrix44 default constructor  */
	HomogeneousMatrix44 *homogeneousTransformation2 = new HomogeneousMatrix44();
	const double* matrixData;
	matrixData = homogeneousTransformation2->getRawData();
	point111->homogeneousTransformation(homogeneousTransformation2);
//	cout << *point111 << endl; // actual

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->z, maxTolerance);
}

void Point3DTest::testRotation90X(){

	AngleAxis<double> rotation(M_PI_2l, Vector3d(1,0,0));
	transformation = rotation;
//	cout << transformation * referenceVector << endl; // expected

	HomogeneousMatrix44 *homogeneousTransformation = new HomogeneousMatrix44(&transformation);
	point111->homogeneousTransformation(homogeneousTransformation);
//	cout << *point111 << endl; // actual

	/* check 90° rotation about X */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, point111->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->z, maxTolerance);
}

void Point3DTest::testRotation90Y(){

	AngleAxis<double> rotation(M_PI_2l, Vector3d(0,1,0));
	transformation = rotation;
//	cout << transformation * referenceVector << endl; // expected

	HomogeneousMatrix44 *homogeneousTransformation = new HomogeneousMatrix44(&transformation);
	point111->homogeneousTransformation(homogeneousTransformation);
//	cout << *point111 << endl; // actual

	/* check 90° rotation about Y */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, point111->z, maxTolerance);
}

void Point3DTest::testRotation90Z(){

	AngleAxis<double> rotation(M_PI_2l, Vector3d(0,0,1));
	transformation = rotation;
//	cout << transformation * referenceVector << endl; // expected

	HomogeneousMatrix44 *homogeneousTransformation = new HomogeneousMatrix44(&transformation);
	point111->homogeneousTransformation(homogeneousTransformation);
//	cout << *point111 << endl; // actual

	/* check 90° rotation about Z */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, point111->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->z, maxTolerance);
}

void Point3DTest::testTranslation(){

	Translation<double,3> translation(1, 1, 1);
	transformation = translation;
//	cout << transformation * referenceVector << endl; // expected

	HomogeneousMatrix44 *homogeneousTransformation = new HomogeneousMatrix44(&transformation);
	point111->homogeneousTransformation(homogeneousTransformation);
//	cout << *point111 << endl; // actual

	/* check translation by (1,1,1)*/
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, point111->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, point111->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, point111->z, maxTolerance);
}

}

/* EOF */
