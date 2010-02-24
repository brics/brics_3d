/**
 * @file 
 * TriangleTest.cpp
 *
 * @date: Feb 24, 2010
 * @author: sblume
 */

#include "TriangleTest.h"
#include <sstream>

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( TriangleTest );

void TriangleTest::setUp() {

	vertex000 = new Point3D;
	vertex111 = new Point3D(1, 1, 1);
	vertexMinus123 = new Point3D(-1.0, -2.0, -3.0);

	defaultTriangle = new Triangle();
	testTriangle = new Triangle (*vertex000, *vertex111, *vertexMinus123);
}

void TriangleTest::tearDown() {
	delete testTriangle;
	delete defaultTriangle;

	delete vertex000;
	delete vertex111;
	delete vertexMinus123;
}

void TriangleTest::testConstructor() {
	Point3D* tmpPoint;

	for (int i = 0; i <= 2; ++i) { //everything should be initialized with 0.0
		tmpPoint = defaultTriangle->getVertex(i);
		CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, tmpPoint->getX(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, tmpPoint->getY(), maxTolerance);
		CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, tmpPoint->getZ(), maxTolerance);
	}


	tmpPoint = testTriangle->getVertex(0);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, tmpPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, tmpPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, tmpPoint->getZ(), maxTolerance);

	tmpPoint = testTriangle->getVertex(1);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, tmpPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, tmpPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, tmpPoint->getZ(), maxTolerance);

	tmpPoint = testTriangle->getVertex(2);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, tmpPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, tmpPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, tmpPoint->getZ(), maxTolerance);

}

void TriangleTest::testModification() {
	Point3D* tmpPoint;

	testTriangle->setVertex(0, Point3D(1,2,3));
	testTriangle->setVertex(1, Point3D(4,5,6));
	testTriangle->setVertex(2, Point3D(7,8,9));

	tmpPoint = testTriangle->getVertex(0);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, tmpPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, tmpPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, tmpPoint->getZ(), maxTolerance);

	tmpPoint = testTriangle->getVertex(1);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, tmpPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, tmpPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, tmpPoint->getZ(), maxTolerance);

	tmpPoint = testTriangle->getVertex(2);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, tmpPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, tmpPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(9.0, tmpPoint->getZ(), maxTolerance);

	tmpPoint->setX(10.0);

	tmpPoint = testTriangle->getVertex(2);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(10.0, tmpPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(8.0, tmpPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(9.0, tmpPoint->getZ(), maxTolerance);


}

void TriangleTest::testTransformation() {

	/* create reference points, as the original points will be modified... */
	Point3D vertex000Copy = Point3D(vertex000);
	Point3D vertex111Copy = Point3D(vertex111);
	Point3D vertexMinus123Copy = Point3D(vertexMinus123);
	Point3D* tmpPoint;

	/*
	 * rotate 90° abouz x axis
	 */
	AngleAxis<double> rotation(M_PI_2l, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
	HomogeneousMatrix44* homogeneousTransformation = new HomogeneousMatrix44(&transformation);

	/* apply transformation to reference points and triangle */
	vertex000Copy.homogeneousTransformation(homogeneousTransformation);
	vertex111Copy.homogeneousTransformation(homogeneousTransformation);
	vertexMinus123Copy.homogeneousTransformation(homogeneousTransformation);
	testTriangle->homogeneousTransformation(homogeneousTransformation);

	/* check 90° rotation about X */
	tmpPoint = testTriangle->getVertex(0);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(vertex000Copy.getX(), tmpPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(vertex000Copy.getY(), tmpPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(vertex000Copy.getZ(), tmpPoint->getZ(), maxTolerance);

	tmpPoint = testTriangle->getVertex(1);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(vertex111Copy.getX(), tmpPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(vertex111Copy.getY(), tmpPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(vertex111Copy.getZ(), tmpPoint->getZ(), maxTolerance);

	tmpPoint = testTriangle->getVertex(2);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(vertexMinus123Copy.getX(), tmpPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(vertexMinus123Copy.getY(), tmpPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(vertexMinus123Copy.getZ(), tmpPoint->getZ(), maxTolerance);

	delete homogeneousTransformation;
}

void TriangleTest::testStreaming() {

	string comparatorString;
	stringstream testStringStream0;

	testStringStream0 << *testTriangle;
	comparatorString = testStringStream0.str();
	CPPUNIT_ASSERT(comparatorString.compare("0 0 0, 1 1 1, -1 -2 -3") == 0);
}


}

/* EOF */
