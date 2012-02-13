/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#include "GenericInterfacesTest.h"


namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( GenericInterfacesTest );

void GenericInterfacesTest::setUp() {
	inputPointCloud1 = new PointCloud3D();
	inputPointCloud1->addPoint(Point3D(0,0,0));
	inputPointCloud1->addPoint(Point3D(0,0,1));
	inputPointCloud1->addPoint(Point3D(0,1,0));
	inputPointCloud1->addPoint(Point3D(0,1,1));
	inputPointCloud1->addPoint(Point3D(1,0,0));
	inputPointCloud1->addPoint(Point3D(1,0,1));
	inputPointCloud1->addPoint(Point3D(1,1,0));
	inputPointCloud1->addPoint(Point3D(1,1,1));
	inputPointCloud1->addPoint(Point3D(1,1,0.9)); //some new points that will be filtered away
	inputPointCloud1->addPoint(Point3D(1,1,0.2));

	inputPointCloud2 = new PointCloud3D();

	/* make a copy */
	stringstream tmpSteam;
	tmpSteam << *inputPointCloud1;
	tmpSteam >> *inputPointCloud2;

	outputPointCloud = new PointCloud3D();
	resultTransform = new HomogeneousMatrix44();
	meshExplicit = new TriangleMeshExplicit();
	meshImplicit = new TriangleMeshImplicit();
}

void GenericInterfacesTest::tearDown() {
	if (inputPointCloud1) {
		delete inputPointCloud1;
		inputPointCloud1 = 0;
	}
	if (inputPointCloud2) {
		delete inputPointCloud2;
		inputPointCloud2 = 0;
	}
	if (outputPointCloud) {
		delete outputPointCloud;
		outputPointCloud = 0;
	}
	if (resultTransform) {
		delete resultTransform;
		resultTransform = 0;
	}
	if (meshExplicit) {
		delete meshExplicit;
		meshExplicit = 0;
	}
	if (meshImplicit) {
		delete meshImplicit;
		meshImplicit = 0;
	}
}

void GenericInterfacesTest::testFiltering() {
	CPPUNIT_ASSERT_EQUAL(10u, inputPointCloud1->getSize());
	CPPUNIT_ASSERT_EQUAL(0u, outputPointCloud->getSize());


	IFiltering* filter = new Octree();
	IOctreeSetup* octreeSetup = dynamic_cast<IOctreeSetup*>(filter);
	octreeSetup->setVoxelSize(0.2);
	filter->filter(inputPointCloud1, outputPointCloud);


	CPPUNIT_ASSERT_EQUAL(10u, inputPointCloud1->getSize());
	CPPUNIT_ASSERT_EQUAL(8u, outputPointCloud->getSize());

}

void GenericInterfacesTest::testRegistration() {

	IRegistration* registrator = new IterativeClosestPoint(
			new PointCorrespondenceKDTree(),
			new RigidTransformationEstimationSVD());
	registrator->match(inputPointCloud1, inputPointCloud2, resultTransform);

	const double* matrixPtr;
	matrixPtr = resultTransform->getRawData();

	/* point clouds are idendical so there should be no translation */
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[12], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[13], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(0.0, matrixPtr[14], maxTolerance);

}

void GenericInterfacesTest::testMeshGeneration() {

	//IMeshGeneration* meshGenerator = new DelaunayTriangulationOSG(); // here we ould need to link to OSG -> but this dependancy should not be in the unit tests
	CPPUNIT_FAIL("TODO");
}

}

/* EOF */
