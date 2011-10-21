/**
 * @file 
 * SceneGraphNodesTest.h
 *
 * @date: Oct 20, 2011
 * @author: sblume
 */

#ifndef SCENEGRAPHNODESTEST_H_
#define SCENEGRAPHNODESTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "core/HomogeneousMatrix44.h"
#include "worldModel/sceneGraph/Node.h"
#include "worldModel/sceneGraph/Group.h"

namespace unitTests {

using namespace BRICS_3D;
using namespace Eigen;
using namespace std;
using namespace BRICS_3D::RSG;
using std::cout;
using std::endl;

class SceneGraphNodesTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( SceneGraphNodesTest );
	CPPUNIT_TEST( testNode );
	CPPUNIT_TEST( testGroup );
	CPPUNIT_TEST( testSimpleGraph );
	CPPUNIT_TEST( testOwnership );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testNode();
	void testGroup();
	void testOwnership();
	void testSimpleGraph();

};

}

#endif /* SCENEGRAPHNODESTEST_H_ */

/* EOF */
