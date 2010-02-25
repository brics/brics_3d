/**
 * @file 
 * TriangleMeshTest.h
 *
 * @date: Feb 25, 2010
 * @author: sblume
 */

#ifndef TRIANGLEMESHTEST_H_
#define TRIANGLEMESHTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "core/TriangleMeshExplicit.h"
#include "core/TriangleMeshImplicit.h"

using namespace BRICS_3D;
using namespace std;


namespace unitTests {

class TriangleMeshTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( TriangleMeshTest );
	CPPUNIT_TEST( testExplicitMeshConstructor );
	CPPUNIT_TEST( testImplicitMeshConstructor );
	CPPUNIT_TEST( testPolymorphMeshConstructor );
	CPPUNIT_TEST( testImplicitMeshModification );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testExplicitMeshConstructor();
	void testImplicitMeshConstructor();
	void testPolymorphMeshConstructor();
	void testImplicitMeshModification();

private:

	Point3D* vertex000;
	Point3D* vertex100;
	Point3D* vertex101;
	Point3D* vertex001;
	Point3D* vertex110;
	Point3D* vertex111;

	Triangle* testTriangle1;
	Triangle* testTriangle2;
	Triangle* testTriangle3;
	Triangle* testTriangle4;

	ITriangleMesh* abstractMesh;

};

}

#endif /* TRIANGLEMESHTEST_H_ */

/* EOF */
