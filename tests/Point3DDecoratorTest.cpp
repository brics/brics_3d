/**
 * @file 
 * Point3DDecoratorTest.cpp
 *
 * @date: Feb 10, 2010
 * @author: sblume
 */

#include "Point3DDecoratorTest.h"

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( Point3DDecoratorTest );

void Point3DDecoratorTest::setUp() {

	maxCoordValue = std::numeric_limits<BRICS_3D::Coordinate>::max();
	minCoordValue = -maxCoordValue;

	point000 = new Point3D;
	point111 = new Point3D(1, 1, 1);
	pointMinus123 = new Point3D(-1.0, -2.0, -3.0);
	pointMax = new Point3D(maxCoordValue, maxCoordValue, maxCoordValue);
	pointMin = new Point3D(minCoordValue, minCoordValue, minCoordValue);

	decoratedPointMinus123 = new ColoredPoint3D(pointMinus123, 4, 5, 6);
}

void Point3DDecoratorTest::tearDown() {
	delete decoratedPointMinus123;
	delete point000;
	delete point111;
	delete pointMinus123;
	delete pointMax;
	delete pointMin;
}

void Point3DDecoratorTest::testColorDecoration() {
	ColoredPoint3D* decoratedPoint = new ColoredPoint3D(point000);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, point000->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, point000->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, point000->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(point000->getX(), decoratedPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point000->getY(), decoratedPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point000->getZ(), decoratedPoint->getZ(), maxTolerance);

	/* re-decorate point */
	decoratedPoint->decorate(point111);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getX(), decoratedPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getY(), decoratedPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getZ(), decoratedPoint->getZ(), maxTolerance);

	/* cast to Point3D */
	Point3D* basePoint = dynamic_cast<Point3D*>(decoratedPoint);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getX(), basePoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getY(), basePoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getZ(), basePoint->getZ(), maxTolerance);

	delete decoratedPoint;

}

void Point3DDecoratorTest::testRecursiveDecoration() {
	ColoredPoint3D* decoratedPointInner = new ColoredPoint3D(point111,1,2,3);
	ColoredPoint3D* decoratedPointOuter = new ColoredPoint3D(decoratedPointInner,4,5,6);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getX(), decoratedPointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getY(), decoratedPointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getZ(), decoratedPointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getX(), decoratedPointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getY(), decoratedPointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getZ(), decoratedPointOuter->getZ(), maxTolerance);

	/* cast to Point3D */
	Point3D* basePointInner = dynamic_cast<Point3D*>(decoratedPointInner);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getX(), basePointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getY(), basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getZ(), basePointInner->getZ(), maxTolerance);

	Point3D* basePointOuter = dynamic_cast<Point3D*>(decoratedPointOuter);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getX(), basePointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getY(), basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getZ(), basePointOuter->getZ(), maxTolerance);

	/*
	 * check if chances are really transparent...
	 */
	Coordinate testX;
	Coordinate testY;
	Coordinate testZ;

	/* change data from outer skin/layer/decorator/wrapper */
	testX = 10.0;
	testY = 11.0;
	testZ = 12.0;

	decoratedPointOuter->setX(testX);
	decoratedPointOuter->setY(testY);
	decoratedPointOuter->setZ(testZ);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX ,point111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, point111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, point111->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX , basePointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX , basePointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointOuter->getZ(), maxTolerance);

	/* change data from inner skin/layer/decorator/wrapper */
	testX = 20.0;
	testY = 21.0;
	testZ = 22.0;

	decoratedPointInner->setX(testX);
	decoratedPointInner->setY(testY);
	decoratedPointInner->setZ(testZ);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX ,point111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, point111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, point111->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX , basePointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX , basePointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointOuter->getZ(), maxTolerance);

	/* change data from core/base */
	testX = 30.0;
	testY = 31.0;
	testZ = 32.0;

	point111->setX(testX);
	point111->setY(testY);
	point111->setZ(testZ);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, point111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, point111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, point111->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, basePointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, basePointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointOuter->getZ(), maxTolerance);

	/*
	 * change data from outer skin/layer/decorator/wrapper,
	 * but manipulate re-casted Point3D
	 */
	testX = 40.0;
	testY = 41.0;
	testZ = 42.0;

	basePointOuter->setX(testX);
	basePointOuter->setY(testY);
	basePointOuter->setZ(testZ);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, point111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, point111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, point111->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, basePointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, basePointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointOuter->getZ(), maxTolerance);

	/*
	 * change data from inner skin/layer/decorator/wrapper,
	 * but manipulate re-casted Point3D
	 */
	testX = 50.0;
	testY = 51.0;
	testZ = 52.0;

	basePointInner->setX(testX);
	basePointInner->setY(testY);
	basePointInner->setZ(testZ);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, point111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, point111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, point111->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, basePointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, basePointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointOuter->getZ(), maxTolerance);

	delete decoratedPointOuter;
	delete decoratedPointInner;

}

void Point3DDecoratorTest::testAddition() {
	Point3D resultPoint;
	Point3D basePointMinus123 = *decoratedPointMinus123;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, decoratedPointMinus123->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, decoratedPointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, decoratedPointMinus123->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, basePointMinus123.getX(), maxTolerance);	//preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, basePointMinus123.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, basePointMinus123.getZ(), maxTolerance);


	resultPoint = *decoratedPointMinus123 + point111;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultPoint.getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, resultPoint.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, resultPoint.getZ(), maxTolerance);

	resultPoint = basePointMinus123 + point111;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, resultPoint.getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, resultPoint.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, resultPoint.getZ(), maxTolerance);

	/* now only decorated points are used*/
	ColoredPoint3D decoratedPoint456(new Point3D(4,5,6),7,8,9);
	Point3D* decoratedResultPoint = new ColoredPoint3D(new Point3D(),13,14,15); //polymorph result

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, decoratedPointMinus123->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, decoratedPointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, decoratedPointMinus123->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, decoratedPoint456.getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, decoratedPoint456.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, decoratedPoint456.getZ(), maxTolerance);


//	decoratedResultPoint = decoratedPoint456 + decoratedPointMinus123;
	*decoratedResultPoint = decoratedPoint456 + decoratedPointMinus123;
//	cout << *decoratedResultPoint << endl;
	ColoredPoint3D* downCastedResult = dynamic_cast<ColoredPoint3D*>(decoratedResultPoint);
//	cout << *downCastedResult << endl;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, decoratedResultPoint->getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, decoratedResultPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, decoratedResultPoint->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, downCastedResult->getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, downCastedResult->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, downCastedResult->getZ(), maxTolerance);

	delete decoratedResultPoint;

}

void Point3DDecoratorTest::testSubtraction() {
	Point3D resultPoint;
	Point3D basePointMinus123 = *decoratedPointMinus123;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, decoratedPointMinus123->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, decoratedPointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, decoratedPointMinus123->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, basePointMinus123.getX(), maxTolerance);	//preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, basePointMinus123.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, basePointMinus123.getZ(), maxTolerance);


	resultPoint = *decoratedPointMinus123 - point111;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, resultPoint.getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, resultPoint.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-4.0, resultPoint.getZ(), maxTolerance);

	resultPoint = basePointMinus123 - point111;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, resultPoint.getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, resultPoint.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-4.0, resultPoint.getZ(), maxTolerance);

	/* now only decorated points are used*/
	ColoredPoint3D decoratedPoint456(new Point3D(4,5,6),7,8,9);
	Point3D* decoratedResultPoint = new ColoredPoint3D(new Point3D(),13,14,15); //polymorph result

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, decoratedPointMinus123->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, decoratedPointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, decoratedPointMinus123->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, decoratedPoint456.getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, decoratedPoint456.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, decoratedPoint456.getZ(), maxTolerance);


	*decoratedResultPoint = decoratedPoint456 - decoratedPointMinus123;
	ColoredPoint3D* downCastedResult = dynamic_cast<ColoredPoint3D*>(decoratedResultPoint);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, decoratedResultPoint->getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, decoratedResultPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(9.0, decoratedResultPoint->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, downCastedResult->getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(7.0, downCastedResult->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(9.0, downCastedResult->getZ(), maxTolerance);

	delete decoratedResultPoint;
}

void Point3DDecoratorTest::testMultiplication() {
	Point3D resultPoint;
	Point3D basePointMinus123 = *decoratedPointMinus123;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, decoratedPointMinus123->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, decoratedPointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, decoratedPointMinus123->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, basePointMinus123.getX(), maxTolerance);	//preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, basePointMinus123.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, basePointMinus123.getZ(), maxTolerance);


	resultPoint = *decoratedPointMinus123 * 10;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-10.0, resultPoint.getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-20.0, resultPoint.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-30.0, resultPoint.getZ(), maxTolerance);

	resultPoint = basePointMinus123 * 100;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-100.0, resultPoint.getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-200.0, resultPoint.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-300.0, resultPoint.getZ(), maxTolerance);

	/* now only decorated points are used*/
	ColoredPoint3D decoratedPoint456(new Point3D(4,5,6),7,8,9);
	Point3D* decoratedResultPoint = new ColoredPoint3D(new Point3D(),13,14,15); //polymorph result

	CPPUNIT_ASSERT_DOUBLES_EQUAL(4.0, decoratedPoint456.getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5.0, decoratedPoint456.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6.0, decoratedPoint456.getZ(), maxTolerance);


	*decoratedResultPoint = decoratedPoint456 * 1000;
	ColoredPoint3D* downCastedResult = dynamic_cast<ColoredPoint3D*>(decoratedResultPoint);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4000.0, decoratedResultPoint->getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5000.0, decoratedResultPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6000.0, decoratedResultPoint->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(4000.0, downCastedResult->getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5000.0, downCastedResult->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6000.0, downCastedResult->getZ(), maxTolerance);

	delete decoratedResultPoint;
}

void Point3DDecoratorTest::testTransfomration() {
	/*
	 * rotate 90° abouz x axis
	 */
	AngleAxis<double> rotation(M_PI_2l, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
//	cout << transformation * referenceVector << endl; // expected

	HomogeneousMatrix44* homogeneousTransformation = new HomogeneousMatrix44(&transformation);
	ColoredPoint3D* decoraredPoint111 = new ColoredPoint3D(point111, 2, 3, 4);
	decoraredPoint111->homogeneousTransformation(homogeneousTransformation);
//	cout << *point111 << endl; // actual

	/* check 90° rotation about X */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, decoraredPoint111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, decoraredPoint111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, decoraredPoint111->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, point111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, point111->getZ(), maxTolerance);

	delete homogeneousTransformation;

	/*
	 * translate by (10, 20, 30)
	 */
	Translation<double,3> translation(10, 20, 30);
	transformation = translation;
//	cout << transformation * referenceVector << endl; // expected

	homogeneousTransformation = new HomogeneousMatrix44(&transformation);
	decoratedPointMinus123->homogeneousTransformation(homogeneousTransformation);
//	cout << *point111 << endl; // actual

	/* check translation by (10, 20, 30) */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(9.0, decoratedPointMinus123->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(18.0, decoratedPointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(27.0, decoratedPointMinus123->getZ(), maxTolerance);

	Point3D* basePointMinus123 = dynamic_cast<Point3D*>(decoratedPointMinus123);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(9.0, basePointMinus123->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(18.0, basePointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(27.0, basePointMinus123->getZ(), maxTolerance);

	/* perform again same translation, but perform on base class */
	basePointMinus123->homogeneousTransformation(homogeneousTransformation);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(19.0, decoratedPointMinus123->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(38.0, decoratedPointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(57.0, decoratedPointMinus123->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(19.0, basePointMinus123->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(38.0, basePointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(57.0, basePointMinus123->getZ(), maxTolerance);

	delete homogeneousTransformation;

}

void Point3DDecoratorTest::testStreaming() {
	ColoredPoint3D* decoratedPointInner = new ColoredPoint3D(pointMinus123,1,2,3);
	ColoredPoint3D* decoratedPointOuter = new ColoredPoint3D(decoratedPointInner,4,5,6);

	/*
	 * check if chances are really transparent...
	 */
	Coordinate testX;
	Coordinate testY;
	Coordinate testZ;

	/* change data from outer skin/layer/decorator/wrapper */
	testX = 10.0;
	testY = 11.0;
	testZ = 12.0;
	decoratedPointOuter->setX(testX);
	decoratedPointOuter->setY(testY);
	decoratedPointOuter->setZ(testZ);

	/* cast to Point3D */
	Point3D* basePointInner = dynamic_cast<Point3D*>(decoratedPointInner);
	Point3D* basePointOuter = dynamic_cast<Point3D*>(decoratedPointOuter);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX , pointMinus123->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, pointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, pointMinus123->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX , decoratedPointOuter->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, decoratedPointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, decoratedPointOuter->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, decoratedPointInner->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, decoratedPointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, decoratedPointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX , basePointOuter->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointOuter->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX , basePointInner->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, basePointInner->getZ(), maxTolerance);

	/*
	 * now its getting intresting: check the stream outputs
	 */
	string comparatorString;
	stringstream testStringStream0;
	stringstream testStringStream1;
	stringstream testStringStream2;
	stringstream testStringStream3;
	stringstream testStringStream4;

//	cout << "testStreaming: "<< endl;
//	cout << "pointMinus123: "  << *pointMinus123 << endl;
//	cout << "decoratedPointOuter: "  << *decoratedPointOuter << endl;
//	cout << "decoratedPointInner: "  << *decoratedPointInner << endl;
//	cout << "basePointOuter: "  << *basePointOuter << endl;
//	cout << "basePointInner: "  << *basePointInner << endl;

	testStringStream0 << *pointMinus123;
	testStringStream1 << *decoratedPointOuter;
	testStringStream2 << *decoratedPointInner;
	testStringStream3 << *basePointOuter;
	testStringStream4 << *basePointInner;

	comparatorString.clear();
	comparatorString = testStringStream0.str();
	CPPUNIT_ASSERT(comparatorString.compare("10 11 12") == 0);

	comparatorString.clear();
	comparatorString = testStringStream1.str();
	CPPUNIT_ASSERT(comparatorString.compare("10 11 12 4 5 6") == 0); //different decoration layers have different output...

	comparatorString.clear();
	comparatorString = testStringStream2.str();
	CPPUNIT_ASSERT(comparatorString.compare("10 11 12 1 2 3") == 0); //different decoration layers have different output...

	comparatorString.clear();
	comparatorString = testStringStream3.str();
	CPPUNIT_ASSERT(comparatorString.compare("10 11 12") == 0);

	comparatorString.clear();
	comparatorString = testStringStream4.str();
	CPPUNIT_ASSERT(comparatorString.compare("10 11 12") == 0);

	/*
	 * test input
	 */
	ColoredPoint3D streamedDecoratedPoint0(new Point3D);
	testStringStream2 >> streamedDecoratedPoint0;
//	cout << streamedDecoratedPoint0 << endl;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, streamedDecoratedPoint0.getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, streamedDecoratedPoint0.getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, streamedDecoratedPoint0.getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1, streamedDecoratedPoint0.red, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2, streamedDecoratedPoint0.green, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3, streamedDecoratedPoint0.blue, maxTolerance);

	Point3D* baseStreamedDecoratedPoint0 = dynamic_cast<Point3D*>(&streamedDecoratedPoint0);
//	cout << *baseStreamedDecoratedPoint0 << endl;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testX, baseStreamedDecoratedPoint0->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testY, baseStreamedDecoratedPoint0->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(testZ, baseStreamedDecoratedPoint0->getZ(), maxTolerance);

	delete basePointOuter;
	delete basePointInner;

}

void Point3DDecoratorTest::testRawAccess() {
	Coordinate* result = new Coordinate[3];

	decoratedPointMinus123->getRawData(result);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(pointMinus123->getX(), result[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(pointMinus123->getY(), result[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(pointMinus123->getZ(), result[2], maxTolerance);

	delete result;

}

}

/* EOF */
