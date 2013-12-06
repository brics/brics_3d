/**
 * @file 
 * BoxROIExtractorTest.cpp
 *
 * @date: Apr 1, 2012
 * @author: sblume
 */

#include "BoxROIExtractorTest.h"

namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( BoxROIExtractorTest );

void BoxROIExtractorTest::setUp() {
	testCloud = new PointCloud3D();
	testCloud->addPoint(Point3D(0,0,0));

	// along x axis
	testCloud->addPoint(Point3D(1,0,0));
	testCloud->addPoint(Point3D(2,0,0));
	testCloud->addPoint(Point3D(-1,0,0));
	testCloud->addPoint(Point3D(-2,0,0));

	// along y axis
	testCloud->addPoint(Point3D(0,1,0));
	testCloud->addPoint(Point3D(0,2,0));
	testCloud->addPoint(Point3D(0,-1,0));
	testCloud->addPoint(Point3D(0,-2,0));

	// along z axis
	testCloud->addPoint(Point3D(0,0,1));
	testCloud->addPoint(Point3D(0,0,2));
	testCloud->addPoint(Point3D(0,0,-1));
	testCloud->addPoint(Point3D(0,0,-2));

}

void BoxROIExtractorTest::tearDown() {
	if (testCloud) {
		delete testCloud;
		testCloud = 0;
	}
}


void BoxROIExtractorTest::testConstructor() {
	BoxROIExtractor defaultFilter;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, defaultFilter.getSizeX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, defaultFilter.getSizeY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, defaultFilter.getSizeZ(), maxTolerance);

	defaultFilter.setSizeX(2);
	defaultFilter.setSizeY(2);
	defaultFilter.setSizeZ(2);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, defaultFilter.getSizeX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, defaultFilter.getSizeY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, defaultFilter.getSizeZ(), maxTolerance);


	boxFilter = new BoxROIExtractor(1,2,3);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, dynamic_cast<BoxROIExtractor*>(boxFilter)->getSizeX(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, dynamic_cast<BoxROIExtractor*>(boxFilter)->getSizeY(), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, dynamic_cast<BoxROIExtractor*>(boxFilter)->getSizeZ(), maxTolerance);

	delete boxFilter;

}

void BoxROIExtractorTest::testBoxExtraction() {
	Coordinate x  = 0.0;
	Coordinate y  = 0.0;
	Coordinate z  = 0.0;

	BoxROIExtractor boxFilter;
	boxFilter.setSizeX(x);
	boxFilter.setSizeY(y);
	boxFilter.setSizeZ(z);

	PointCloud3D* resultPointCloud = new PointCloud3D();

	// 0 size
	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize()); // preconditions
	CPPUNIT_ASSERT_EQUAL(0u, resultPointCloud->getSize());

	boxFilter.filter(testCloud, resultPointCloud);

	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize()); //postcondiitons
	CPPUNIT_ASSERT_EQUAL(1u, resultPointCloud->getSize());


	// size = 1
	resultPointCloud->getPointCloud()->clear(); //preconditions
	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize());
	CPPUNIT_ASSERT_EQUAL(0u, resultPointCloud->getSize());

	x = 1;
	y = 1;
	z = 1;
	boxFilter.setSizeX(x);
	boxFilter.setSizeY(y);
	boxFilter.setSizeZ(z);

	boxFilter.filter(testCloud, resultPointCloud);

	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize()); //postcondiitons
	CPPUNIT_ASSERT_EQUAL(1u, resultPointCloud->getSize());

	// size = 1.5
	resultPointCloud->getPointCloud()->clear(); //preconditions
	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize());
	CPPUNIT_ASSERT_EQUAL(0u, resultPointCloud->getSize());

	x = 1.5;
	y = 1.5;
	z = 1.5;
	boxFilter.setSizeX(x);
	boxFilter.setSizeY(y);
	boxFilter.setSizeZ(z);

	boxFilter.filter(testCloud, resultPointCloud);

	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize()); //postcondiitons
	CPPUNIT_ASSERT_EQUAL(1u, resultPointCloud->getSize());

	// size = 2.0
	resultPointCloud->getPointCloud()->clear(); //preconditions
	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize());
	CPPUNIT_ASSERT_EQUAL(0u, resultPointCloud->getSize());

	x = 2.0;
	y = 2.0;
	z = 2.0;
	boxFilter.setSizeX(x);
	boxFilter.setSizeY(y);
	boxFilter.setSizeZ(z);

	boxFilter.filter(testCloud, resultPointCloud);
//	cout << "Box filtered coud"<< endl;
//	cout << *resultPointCloud << endl;
	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize()); //postcondiitons
	CPPUNIT_ASSERT_EQUAL(7u, resultPointCloud->getSize());

	// size = 2.5
	resultPointCloud->getPointCloud()->clear(); //preconditions
	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize());
	CPPUNIT_ASSERT_EQUAL(0u, resultPointCloud->getSize());

	x = 2.5;
	y = 2.5;
	z = 2.5;
	boxFilter.setSizeX(x);
	boxFilter.setSizeY(y);
	boxFilter.setSizeZ(z);

	boxFilter.filter(testCloud, resultPointCloud);

	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize()); //postcondiitons
	CPPUNIT_ASSERT_EQUAL(7u, resultPointCloud->getSize());

	// size = 4.0 -> this is the smales values to contain all values...
	resultPointCloud->getPointCloud()->clear(); //preconditions
	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize());
	CPPUNIT_ASSERT_EQUAL(0u, resultPointCloud->getSize());

	x = 4.0;
	y = 4.0;
	z = 4.0;
	boxFilter.setSizeX(x);
	boxFilter.setSizeY(y);
	boxFilter.setSizeZ(z);

	boxFilter.filter(testCloud, resultPointCloud);

	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize()); //postcondiitons
	CPPUNIT_ASSERT_EQUAL(13u, resultPointCloud->getSize());

	// size = 4.5
	resultPointCloud->getPointCloud()->clear(); //preconditions
	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize());
	CPPUNIT_ASSERT_EQUAL(0u, resultPointCloud->getSize());

	x = 4.5;
	y = 4.5;
	z = 4.5;
	boxFilter.setSizeX(x);
	boxFilter.setSizeY(y);
	boxFilter.setSizeZ(z);

	boxFilter.filter(testCloud, resultPointCloud);

	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize()); //postcondiitons
	CPPUNIT_ASSERT_EQUAL(13u, resultPointCloud->getSize());

	HomogeneousMatrix44::IHomogeneousMatrix44Ptr identity(new HomogeneousMatrix44());
	boxFilter.setBoxOrigin(identity);


	// size = 4.5
	resultPointCloud->getPointCloud()->clear(); //preconditions
	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize());
	CPPUNIT_ASSERT_EQUAL(0u, resultPointCloud->getSize());

	x = 4.5;
	y = 4.5;
	z = 4.5;
	boxFilter.setSizeX(x);
	boxFilter.setSizeY(y);
	boxFilter.setSizeZ(z);

	boxFilter.filter(testCloud, resultPointCloud);

	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize()); //postcondiitons
	CPPUNIT_ASSERT_EQUAL(13u, resultPointCloud->getSize());


	HomogeneousMatrix44::IHomogeneousMatrix44Ptr translation(new HomogeneousMatrix44(1,0,0, 0,1,0, 0,0,1, 10,20,30));
	boxFilter.setBoxOrigin(translation);


	// size = 4.5
	resultPointCloud->getPointCloud()->clear(); //preconditions
	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize());
	CPPUNIT_ASSERT_EQUAL(0u, resultPointCloud->getSize());

	x = 4.5;
	y = 4.5;
	z = 4.5;
	boxFilter.setSizeX(x);
	boxFilter.setSizeY(y);
	boxFilter.setSizeZ(z);

	boxFilter.filter(testCloud, resultPointCloud);

	CPPUNIT_ASSERT_EQUAL(13u, testCloud->getSize()); //postcondiitons
	CPPUNIT_ASSERT_EQUAL(0u, resultPointCloud->getSize());

	delete resultPointCloud;
}

}


/* EOF */
