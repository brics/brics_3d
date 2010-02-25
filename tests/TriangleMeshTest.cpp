/**
 * @file 
 * TriangleMeshTest.cpp
 *
 * @date: Feb 25, 2010
 * @author: sblume
 */

#include "TriangleMeshTest.h"

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( TriangleMeshTest );


void TriangleMeshTest::setUp() {

	/* only to get a rough idea:
	 *
	 *       z       y
	 *       ^      /
	 *       |     /     *(1,1,1)
	 *       |  (1,0,1). .
	 *(0,0,1)* . /. . * 4.
	 *       | 2/  .  .  .
	 *       | / .    .3 *(1,1,0)
	 *       |/.   1  . .
	 *       *--------*-------------> x
	 *    (0,0,0)   (1,0,0)
	 *
	 */

	vertex000 = new Point3D(0,0,0);
	vertex100 = new Point3D(1,0,0);
	vertex101 = new Point3D(1,0,1);
	vertex001 = new Point3D(0,0,1);
	vertex110 = new Point3D(1,1,0);
	vertex111 = new Point3D(1,1,1);

	testTriangle1 = new Triangle(*vertex000, *vertex100, *vertex101);
	testTriangle2 = new Triangle(*vertex101, *vertex001, *vertex000);
	testTriangle3 = new Triangle(*vertex100, *vertex110, *vertex111);
	testTriangle4 = new Triangle(*vertex100, *vertex110, *vertex111);
}

void TriangleMeshTest::tearDown() {

	delete testTriangle1;
	delete testTriangle2;
	delete testTriangle3;
	delete testTriangle4;

	delete vertex000;
	delete vertex100;
	delete vertex101;
	delete vertex001;
	delete vertex110;
	delete vertex111;
}

void TriangleMeshTest::testExplicitConstructor() {
	TriangleMeshExplicit* mesh = new TriangleMeshExplicit();

	CPPUNIT_ASSERT_EQUAL(0, mesh->getSize());
	mesh->addTriangle(testTriangle1);
	CPPUNIT_ASSERT_EQUAL(1, mesh->getSize());
	mesh->addTriangle(testTriangle2);
	CPPUNIT_ASSERT_EQUAL(2, mesh->getSize());
	mesh->addTriangle(testTriangle3);
	CPPUNIT_ASSERT_EQUAL(3, mesh->getSize());
	mesh->addTriangle(testTriangle4);
	CPPUNIT_ASSERT_EQUAL(4, mesh->getSize());

	delete mesh;
}

void TriangleMeshTest::testImplicitConstructor() {
	TriangleMeshImplicit* mesh = new TriangleMeshImplicit();

	CPPUNIT_ASSERT_EQUAL(0, mesh->getSize());
	mesh->addTriangle(*vertex000, *vertex100, *vertex101);
	CPPUNIT_ASSERT_EQUAL(1, mesh->getSize());
	mesh->addTriangle(*vertex101, *vertex001, *vertex000);
	CPPUNIT_ASSERT_EQUAL(2, mesh->getSize());
	mesh->addTriangle(*vertex100, *vertex110, *vertex111);
	CPPUNIT_ASSERT_EQUAL(3, mesh->getSize());
	mesh->addTriangle(*vertex100, *vertex110, *vertex111);
	CPPUNIT_ASSERT_EQUAL(4, mesh->getSize());

	delete mesh;
}

void TriangleMeshTest::testPolymorphConstructor() {
	abstractMesh = new TriangleMeshExplicit();

	CPPUNIT_ASSERT_EQUAL(0, abstractMesh->getSize());
	abstractMesh->addTriangle(*vertex000, *vertex100, *vertex101);
	CPPUNIT_ASSERT_EQUAL(1, abstractMesh->getSize());
	abstractMesh->addTriangle(*vertex101, *vertex001, *vertex000);
	CPPUNIT_ASSERT_EQUAL(2, abstractMesh->getSize());
	abstractMesh->addTriangle(*vertex100, *vertex110, *vertex111);
	CPPUNIT_ASSERT_EQUAL(3, abstractMesh->getSize());
	abstractMesh->addTriangle(*vertex100, *vertex110, *vertex111);
	CPPUNIT_ASSERT_EQUAL(4, abstractMesh->getSize());

	delete abstractMesh;
	abstractMesh = new TriangleMeshImplicit();

	CPPUNIT_ASSERT_EQUAL(0, abstractMesh->getSize());
	abstractMesh->addTriangle(*vertex000, *vertex100, *vertex101);
	CPPUNIT_ASSERT_EQUAL(1, abstractMesh->getSize());
	abstractMesh->addTriangle(*vertex101, *vertex001, *vertex000);
	CPPUNIT_ASSERT_EQUAL(2, abstractMesh->getSize());
	abstractMesh->addTriangle(*vertex100, *vertex110, *vertex111);
	CPPUNIT_ASSERT_EQUAL(3, abstractMesh->getSize());
	abstractMesh->addTriangle(*vertex100, *vertex110, *vertex111);
	CPPUNIT_ASSERT_EQUAL(4, abstractMesh->getSize());

	delete abstractMesh;
}

}

/* EOF */
