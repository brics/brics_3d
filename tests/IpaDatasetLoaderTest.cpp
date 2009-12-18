/**
 * @file 
 * IpaDatasetLoaderTest.cpp
 *
 * @date: Dec 18, 2009
 * @author: sblume
 */

#include "IpaDatasetLoaderTest.h"
#include "util/OSGPointCloudVisualizer.h"

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( IpaDatasetLoaderTest );

void IpaDatasetLoaderTest::setUp() {

	char defaultFilename[255] = { BRICS_IMGAGES_DIR };
	strcat(defaultFilename, "/pringlesGreenMedian_0\0");
	filename = defaultFilename;
	loader = 0;
}

void IpaDatasetLoaderTest::tearDown() {
	if (loader != 0) {
		delete loader;
	}
}

void IpaDatasetLoaderTest::testConstructor() {

	CPPUNIT_ASSERT(loader == 0);
	loader = new IpaDatasetLoader();
	CPPUNIT_ASSERT(loader != 0);

}

void IpaDatasetLoaderTest::testLoadData() {
	loader = new IpaDatasetLoader();

	int row = 200;
	int col = 200;
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	unsigned char red;
	unsigned char green;
	unsigned char blue;


	/* check with wrong path */
	CPPUNIT_ASSERT(loader->loadColoredPointCloud("wrongRathAndFileName") == false);
	CPPUNIT_ASSERT(loader->getData(row, col, x, y, z, red, green, blue) == false);

	/* check with correct path */
	CPPUNIT_ASSERT(loader->loadColoredPointCloud(filename) == true);
	CPPUNIT_ASSERT(loader->getData(row, col, x, y, z, red, green, blue) == true);
	cout << x << " " << y << " " << z << endl;
	cout << static_cast<int>(red) << " " << static_cast<int>(green) << " " << static_cast<int>(blue) << endl;

	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.030823, x, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0123498, y, maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.499429, z, maxTolerance);

	CPPUNIT_ASSERT_DOUBLES_EQUAL(125, static_cast<int>(red) , maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(98, static_cast<int>(green), maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(40, static_cast<int>(blue), maxTolerance);

	/* try with some mean values */
	row = 200000; //would be a 110GiB Image ...
	col = 200000;
	CPPUNIT_ASSERT(loader->getData(row, col, x, y, z, red, green, blue) == false);

	row = 200000;
	col = -200000;
	CPPUNIT_ASSERT(loader->getData(row, col, x, y, z, red, green, blue) == false);

	row = -200000;
	col = -200000;
	CPPUNIT_ASSERT(loader->getData(row, col, x, y, z, red, green, blue) == false);

	row = -200000;
	col = 200000;
	CPPUNIT_ASSERT(loader->getData(row, col, x, y, z, red, green, blue) == false);

	/* at least one parameter is correct */
	row = 0;
	col = 200000;
	CPPUNIT_ASSERT(loader->getData(row, col, x, y, z, red, green, blue) == false);

	row = 0;
	col = -200000;
	CPPUNIT_ASSERT(loader->getData(row, col, x, y, z, red, green, blue) == false);

	row = -200000;
	col = 0;
	CPPUNIT_ASSERT(loader->getData(row, col, x, y, z, red, green, blue) == false);

	row = 200000;
	col = 0;
	CPPUNIT_ASSERT(loader->getData(row, col, x, y, z, red, green, blue) == false);

}

void IpaDatasetLoaderTest::testLoadPointCloud() {
	loader = new IpaDatasetLoader();
	CPPUNIT_ASSERT(loader->loadColoredPointCloud(filename) == true);

	PointCloud3D* resultCloud = 0;
	resultCloud = loader->getPointCloud();
//	cout << resultCloud->getSize() << endl;
	CPPUNIT_ASSERT_EQUAL(28611u, resultCloud->getSize());
}

}

/* EOF */
