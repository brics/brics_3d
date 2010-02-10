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


//	cout << *point111 << endl;
//	cout << *decoratedPointInner << endl;
//	cout << *decoratedPointOuter << endl;
//	cout << *basePointOuter  << endl; //FIXME
//	cout << *basePointInner << endl << endl; //FIXME

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
	cout << "[Test] " << endl;
	cout << *decoratedResultPoint << endl;
	ColoredPoint3D* downCastedResult = dynamic_cast<ColoredPoint3D*>(decoratedResultPoint);
	cout << *downCastedResult << endl;
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, decoratedResultPoint->getX(), maxTolerance); //postconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, decoratedResultPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, decoratedResultPoint->getZ(), maxTolerance);




	cout << resultPoint << endl;
	cout << *decoratedPointMinus123 << endl;
	cout << basePointMinus123 << endl;

}

void Point3DDecoratorTest::testSubtraction() {
	CPPUNIT_FAIL("TODO");
}

void Point3DDecoratorTest::testMultiplication() {
	CPPUNIT_FAIL("TODO");
}

void Point3DDecoratorTest::testTransfomration() {
	CPPUNIT_FAIL("TODO");
}

void Point3DDecoratorTest::testStreaming() {
	CPPUNIT_FAIL("TODO");
}


}

/* EOF */
