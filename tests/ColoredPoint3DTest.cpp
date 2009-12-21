/**
 * @file 
 * ColoredPoint3DTest.cpp
 *
 * @date: Dec 21, 2009
 * @author: sblume
 */

#include "ColoredPoint3DTest.h"

#include <limits>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "Point3DTest.h"
#include "core/HomogeneousMatrix44.h"


namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( ColoredPoint3DTest );

void ColoredPoint3DTest::setUp() {

	/* use absolute max/min of double value */
	maxDouble = std::numeric_limits<double>::max();
	//minDouble = std::numeric_limits<double>::min();
	minDouble = -maxDouble;

	point000 = new ColoredPoint3D;
	point111 = new ColoredPoint3D(1, 1, 1, 1, 1, 1);
	pointMinus123 = new ColoredPoint3D(-1.0, -2.0, -3.00, 1, 1, 1);
	pointMax = new ColoredPoint3D(maxDouble, maxDouble, maxDouble, 1, 1, 1);
	pointMin = new ColoredPoint3D(minDouble, minDouble, minDouble, 1, 1, 1);

	referenceVector << 1, 1, 1;
}

void ColoredPoint3DTest::tearDown() {
	delete point000;
	delete point111;
	delete pointMinus123;
	delete pointMax;
	delete pointMin;
}

void ColoredPoint3DTest::testConstructor() {

	/*
	 * Test if all instances are created correctly.
	 */
	CPPUNIT_ASSERT((point000->x - 0) < maxTolerance);
	CPPUNIT_ASSERT((point000->x - 0) < maxTolerance);
	CPPUNIT_ASSERT((point000->x - 0) < maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, point000->x, maxTolerance); //same as before but with CppUnit macro
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, point000->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, point000->z, maxTolerance);
	CPPUNIT_ASSERT(point000->red == 0);
	CPPUNIT_ASSERT(point000->green == 0);
	CPPUNIT_ASSERT(point000->blue == 0);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->z, maxTolerance);
	CPPUNIT_ASSERT(point111->red == 1);
	CPPUNIT_ASSERT(point111->green == 1);
	CPPUNIT_ASSERT(point111->blue == 1);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, pointMinus123->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, pointMinus123->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, pointMinus123->z, maxTolerance);
	CPPUNIT_ASSERT(pointMinus123->red == 1);
	CPPUNIT_ASSERT(pointMinus123->green == 1);
	CPPUNIT_ASSERT(pointMinus123->blue == 1);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, pointMax->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, pointMax->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(maxDouble, pointMax->z, maxTolerance);
	CPPUNIT_ASSERT(pointMax->red == 1);
	CPPUNIT_ASSERT(pointMax->green == 1);
	CPPUNIT_ASSERT(pointMax->blue == 1);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, pointMin->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, pointMin->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(minDouble, pointMin->z, maxTolerance);
	CPPUNIT_ASSERT(pointMin->red == 1);
	CPPUNIT_ASSERT(pointMin->green == 1);
	CPPUNIT_ASSERT(pointMin->blue == 1);

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


	ColoredPoint3D *newPoint2;
	newPoint2 = new ColoredPoint3D(point111);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint2->x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint2->y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint2->z, maxTolerance);
	CPPUNIT_ASSERT(newPoint2->red == 1);
	CPPUNIT_ASSERT(newPoint2->green == 1);
	CPPUNIT_ASSERT(newPoint2->blue == 1);
	delete newPoint2;

	ColoredPoint3D newPoint3(point111);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint3.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint3.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, newPoint3.z, maxTolerance);
	CPPUNIT_ASSERT(newPoint3.red == 1);
	CPPUNIT_ASSERT(newPoint3.green == 1);
	CPPUNIT_ASSERT(newPoint3.blue == 1);

}


void ColoredPoint3DTest::testStreaming() {
	string comparatorString;
	stringstream testStringStream0;
	stringstream testStringStream1;
	ColoredPoint3D streamedPoint111;
	ColoredPoint3D streamedPointMinus123;

	/* test output */
//	cout << "RGB point = " << *point111 << endl;
	testStringStream0 << *point111;
	comparatorString = testStringStream0.str();
	CPPUNIT_ASSERT(comparatorString.compare("1 1 1 1 1 1") == 0);

	comparatorString.clear();
	testStringStream1 << *pointMinus123;
	comparatorString = testStringStream1.str();
	CPPUNIT_ASSERT(comparatorString.compare("-1 -2 -3 1 1 1") == 0);

	/* test input */
	testStringStream0 >> streamedPoint111;
//	cout << "RGB point 2 = " << streamedPoint111 << endl;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, streamedPoint111.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, streamedPoint111.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, streamedPoint111.z, maxTolerance);
	CPPUNIT_ASSERT(streamedPoint111.red == 1);
	CPPUNIT_ASSERT(streamedPoint111.green == 1);
	CPPUNIT_ASSERT(streamedPoint111.blue == 1);

	testStringStream1 >> streamedPointMinus123;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, streamedPointMinus123.x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, streamedPointMinus123.y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, streamedPointMinus123.z, maxTolerance);
	CPPUNIT_ASSERT(streamedPointMinus123.red == 1);
	CPPUNIT_ASSERT(streamedPointMinus123.green == 1);
	CPPUNIT_ASSERT(streamedPointMinus123.blue == 1);
}

void ColoredPoint3DTest::testIdentity() {

	transformation = Transform3d::Identity();


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

	CPPUNIT_ASSERT(point111->red == 1);
	CPPUNIT_ASSERT(point111->green == 1);
	CPPUNIT_ASSERT(point111->blue == 1);
}

void ColoredPoint3DTest::testTranslation() {

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

	CPPUNIT_ASSERT(point111->red == 1);
	CPPUNIT_ASSERT(point111->green == 1);
	CPPUNIT_ASSERT(point111->blue == 1);
}

}


/* EOF */
