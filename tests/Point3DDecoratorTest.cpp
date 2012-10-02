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

	maxCoordValue = std::numeric_limits<brics_3d::Coordinate>::max();
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
	if (point111) { // we have to be careful as we plan to pass over ownership in the decorator test
		//delete point111;
		point111 = 0;
	}
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
	decoratedPoint->decorate(point111); //with this one you pass by the owner ship to the point

	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getX(), decoratedPoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getY(), decoratedPoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getZ(), decoratedPoint->getZ(), maxTolerance);

	/* cast to Point3D */
	Point3D* basePoint = dynamic_cast<Point3D*>(decoratedPoint);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getX(), basePoint->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getY(), basePoint->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(point111->getZ(), basePoint->getZ(), maxTolerance);

	delete decoratedPoint;
	point111 = 0;

}

void Point3DDecoratorTest::testRecursiveDecoration() {
	ColoredPoint3D* decoratedPointInner = new ColoredPoint3D(point111,1,2,3); //decorator constructor
	decoratedPointInner->decorate(point111); //this means real decoration without creation of a copy; actually overrides value from constructor
	ColoredPoint3D* decoratedPointOuter = new ColoredPoint3D(decoratedPointInner,4,5,6); //decorator constructor
	decoratedPointOuter->decorate(decoratedPointInner);
	ColoredPoint3D* decoratedPointOuterCopy = new ColoredPoint3D(decoratedPointOuter); // _copy_ constructor

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

//	delete decoratedPointOuterCopy;
//	delete decoratedPointInner;
	point111 = 0; //already deleted earlier
}

void Point3DDecoratorTest::testRecursiveDecorationCopies() {
	ColoredPoint3D* decoratedPointInner = new ColoredPoint3D(point111,1,2,3); //decorator constructor
	ColoredPoint3D* decoratedPointOuter = new ColoredPoint3D(decoratedPointInner,4,5,6); //decorator constructor
	ColoredPoint3D* decoratedPointOuterCopy = new ColoredPoint3D(decoratedPointOuter); // _copy_ constructor

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
	Coordinate test2X = 20.0;
	Coordinate test2Y = 21.0;
	Coordinate test2Z = 22.0;

	decoratedPointInner->setX(test2X);
	decoratedPointInner->setY(test2Y);
	decoratedPointInner->setZ(test2Z);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test2X ,point111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test2Y, point111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test2Z, point111->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test2X , basePointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test2Y, basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test2Z, basePointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test2X , basePointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test2Y, basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test2Z, basePointOuter->getZ(), maxTolerance);

	/* change data from core/base */
	Coordinate test3X = 30.0;
	Coordinate test3Y = 31.0;
	Coordinate test3Z = 32.0;

	point111->setX(test3X);
	point111->setY(test3Y);
	point111->setZ(test3Z);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test3X, point111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test3Y, point111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test3Z, point111->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test3X, basePointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test3Y, basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test3Z, basePointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test3X, basePointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test3Y, basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test3Z, basePointOuter->getZ(), maxTolerance);

	/*
	 * change data from outer skin/layer/decorator/wrapper,
	 * but manipulate re-casted Point3D
	 */
	Coordinate test4X = 40.0;
	Coordinate test4Y = 41.0;
	Coordinate test4Z = 42.0;

	basePointOuter->setX(test4X);
	basePointOuter->setY(test4Y);
	basePointOuter->setZ(test4Z);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test4X, point111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test4Y, point111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test4Z, point111->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test4X, basePointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test4Y, basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test4Z, basePointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test4X, basePointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test4Y, basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test4Z, basePointOuter->getZ(), maxTolerance);

	/*
	 * change data from inner skin/layer/decorator/wrapper,
	 * but manipulate re-casted Point3D
	 */
	Coordinate test5X = 50.0;
	Coordinate test5Y = 51.0;
	Coordinate test5Z = 52.0;

	basePointInner->setX(test5X);
	basePointInner->setY(test5Y);
	basePointInner->setZ(test5Z);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test5X, point111->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test5Y, point111->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test5Z, point111->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test5X, basePointInner->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test5Y, basePointInner->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test5Z, basePointInner->getZ(), maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(test5X, basePointOuter->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test5Y, basePointOuter->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(test5Z, basePointOuter->getZ(), maxTolerance);

	delete decoratedPointOuter;

//	delete decoratedPointOuterCopy;
//	delete decoratedPointInner;
//	point111 = 0; //already deleted earlier
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
	AngleAxis<double> rotation(M_PI_2, Vector3d(1,0,0));
	Transform3d transformation;
	transformation = rotation;
//	cout << transformation * referenceVector << endl; // expected

	HomogeneousMatrix44* homogeneousTransformation = new HomogeneousMatrix44(&transformation);
	ColoredPoint3D* decoraredPoint111 = new ColoredPoint3D(point111, 2, 3, 4);
	decoraredPoint111->decorate(point111);
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

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, decoratedPointMinus123->getX(), maxTolerance); //preconditions
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, decoratedPointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, decoratedPointMinus123->getZ(), maxTolerance);

	ColoredPoint3D* decoratedPointInner = new ColoredPoint3D(new Point3D(pointMinus123),1,2,3); //pointMinus123 is already owned by decoratedPointMinus123
	ColoredPoint3D* decoratedPointOuter = new ColoredPoint3D(decoratedPointInner,4,5,6);
	decoratedPointOuter->decorate(decoratedPointInner);

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

	CPPUNIT_ASSERT_DOUBLES_EQUAL(-1.0, decoratedPointMinus123->getX(), maxTolerance); //preconditions (it shold not change as the constructor makes copies)
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2.0, decoratedPointMinus123->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-3.0, decoratedPointMinus123->getZ(), maxTolerance);

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
	CPPUNIT_ASSERT(comparatorString.compare("-1 -2 -3") == 0);

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
//	delete basePointInner; //not necessary any more as basePointOuter will delete the inner pointer automatically
}

void Point3DDecoratorTest::testRawAccess() {
	Coordinate* result = new Coordinate[3];

	decoratedPointMinus123->getRawData(result);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(pointMinus123->getX(), result[0], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(pointMinus123->getY(), result[1], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(pointMinus123->getZ(), result[2], maxTolerance);

	delete result;

}

void Point3DDecoratorTest::testClone() {
	Coordinate valueX1 = 1;
	Coordinate valueY1 = 2;
	Coordinate valueZ1 = 3;
	unsigned char valueR1 = 4;
	unsigned char valueG1 = 5;
	unsigned char valueB1 = 6;
	Point3D* point1 = new Point3D(valueX1, valueY1, valueZ1);
	ColoredPoint3D* coloredPoint1 = new ColoredPoint3D(point1, valueR1, valueG1, valueB1);


	Point3D* testPoint1 = new Point3DIntensity(coloredPoint1, 50.3);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueX1, testPoint1->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueY1, testPoint1->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueZ1, testPoint1->getZ(), maxTolerance);
	CPPUNIT_ASSERT_EQUAL(valueR1, testPoint1->asColoredPoint3D()->getR());
	CPPUNIT_ASSERT_EQUAL(valueG1, testPoint1->asColoredPoint3D()->getG());
	CPPUNIT_ASSERT_EQUAL(valueB1, testPoint1->asColoredPoint3D()->getB());

	Point3D* testPoint2 = testPoint1;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueX1, testPoint2->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueY1, testPoint2->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueZ1, testPoint2->getZ(), maxTolerance);
	CPPUNIT_ASSERT(testPoint2->asColoredPoint3D() != 0);
	CPPUNIT_ASSERT_EQUAL(valueR1, testPoint2->asColoredPoint3D()->getR());
	CPPUNIT_ASSERT_EQUAL(valueG1, testPoint2->asColoredPoint3D()->getG());
	CPPUNIT_ASSERT_EQUAL(valueB1, testPoint2->asColoredPoint3D()->getB());

	Point3D* testPoint3 = testPoint1->clone();

	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueX1, testPoint3->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueY1, testPoint3->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueZ1, testPoint3->getZ(), maxTolerance);
	CPPUNIT_ASSERT(testPoint3->asColoredPoint3D() != 0);
	CPPUNIT_ASSERT_EQUAL(valueR1, testPoint3->asColoredPoint3D()->getR());
	CPPUNIT_ASSERT_EQUAL(valueG1, testPoint3->asColoredPoint3D()->getG());
	CPPUNIT_ASSERT_EQUAL(valueB1, testPoint3->asColoredPoint3D()->getB());

	/* manipulate the original data -> the clone shold not be affected */
	Coordinate valueX2 =7;
	Coordinate valueY2 = 8;
	Coordinate valueZ2 = 9;
	unsigned char valueR2 = 10;
	unsigned char valueG2 = 11;
	unsigned char valueB2 = 12;

	point1->setX(valueX2);
	point1->setY(valueY2);
	point1->setZ(valueZ2);
	coloredPoint1->setR(valueR2);
	coloredPoint1->setG(valueG2);
	coloredPoint1->setB(valueB2);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueX2, testPoint1->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueY2, testPoint1->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueZ2, testPoint1->getZ(), maxTolerance);
	CPPUNIT_ASSERT_EQUAL(valueR2, testPoint1->asColoredPoint3D()->getR());
	CPPUNIT_ASSERT_EQUAL(valueG2, testPoint1->asColoredPoint3D()->getG());
	CPPUNIT_ASSERT_EQUAL(valueB2, testPoint1->asColoredPoint3D()->getB());

	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueX2, testPoint2->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueY2, testPoint2->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueZ2, testPoint2->getZ(), maxTolerance);
	CPPUNIT_ASSERT(testPoint2->asColoredPoint3D() != 0);
	CPPUNIT_ASSERT_EQUAL(valueR2, testPoint2->asColoredPoint3D()->getR());
	CPPUNIT_ASSERT_EQUAL(valueG2, testPoint2->asColoredPoint3D()->getG());
	CPPUNIT_ASSERT_EQUAL(valueB2, testPoint2->asColoredPoint3D()->getB());

	/* should not be affected as it a cloned copy */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueX1, testPoint3->getX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueY1, testPoint3->getY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(valueZ1, testPoint3->getZ(), maxTolerance);
	CPPUNIT_ASSERT(testPoint3->asColoredPoint3D() != 0);
	CPPUNIT_ASSERT_EQUAL(valueR1, testPoint3->asColoredPoint3D()->getR());
	CPPUNIT_ASSERT_EQUAL(valueG1, testPoint3->asColoredPoint3D()->getG());
	CPPUNIT_ASSERT_EQUAL(valueB1, testPoint3->asColoredPoint3D()->getB());

}

}

/* EOF */
