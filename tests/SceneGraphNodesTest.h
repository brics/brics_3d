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
#include "worldModel/sceneGraph/Transform.h"
#include "worldModel/sceneGraph/GeometricNode.h"
#include "worldModel/sceneGraph/Box.h"
#include "worldModel/sceneGraph/Cylinder.h"
#include "worldModel/sceneGraph/INodeVisitor.h"
#include "worldModel/sceneGraph/PathCollector.h"
#include "worldModel/sceneGraph/AttributeFinder.h"
#include "worldModel/sceneGraph/SimpleIdGenerator.h"
#include "worldModel/sceneGraph/SceneGraphFacade.h"
#include "worldModel/sceneGraph/OutdatedDataDeleter.h"

#include "util/Timer.h"

namespace unitTests {

using namespace BRICS_3D;
using namespace Eigen;
using namespace std;
using namespace BRICS_3D::RSG;
using std::cout;
using std::endl;

/*
 * Example how to implement custom visitors.
 */
class IdCollector : public INodeVisitor {
public:
	void visit(Node* node){
		cout << "Node ID  = " << node->getId() << endl;
		collectedIDs.push_back(node->getId());
	};
	void visit(Group* node){
		cout << "Group ID = " << node->getId() << endl;
		collectedIDs.push_back(node->getId());
	};
	void visit(RSG::Transform* node){
		cout << "Transform ID = " << node->getId() << endl;
		collectedIDs.push_back(node->getId());
	};
	void visit(RSG::GeometricNode* node){
		cout << "GeometricNode ID = " << node->getId() << endl;
		collectedIDs.push_back(node->getId());
	};

	std::vector<unsigned int> collectedIDs;
};

class SceneGraphNodesTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( SceneGraphNodesTest );
	CPPUNIT_TEST( testNode );
	CPPUNIT_TEST( testGroup );
	CPPUNIT_TEST( testTransform );
	CPPUNIT_TEST( testTemporalTransform);
	CPPUNIT_TEST( testTemporalTransformAccess);
	CPPUNIT_TEST( testGeometricNode );
	CPPUNIT_TEST( testSimpleGraph );
	CPPUNIT_TEST( testOwnership );
	CPPUNIT_TEST( testSimpleVisitor );
	CPPUNIT_TEST( testPathCollectorVisitor );
	CPPUNIT_TEST( testTransformVisitor );
	CPPUNIT_TEST( testGlobalTransformCalculation );
	CPPUNIT_TEST( testAttributeFinder );
	CPPUNIT_TEST( testOutdatedDataDeleter );
	CPPUNIT_TEST( testIdGenerator );
	CPPUNIT_TEST( testSceneGraphFacade );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testNode();
	void testGroup();
	void testTransform();
	void testTemporalTransform();
	void testTemporalTransformAccess();
	void testGeometricNode();
	void testOwnership();
	void testSimpleGraph();
	void testSimpleVisitor();
	void testPathCollectorVisitor();
	void testTransformVisitor();
	void testGlobalTransformCalculation();
	void testAttributeFinder();
	void testOutdatedDataDeleter();
	void testIdGenerator();
	void testSceneGraphFacade();

private:
	  /// Maximum deviation for equality check of double variables
	  static const double maxTolerance = 0.00001;

	  /// Pointer to matrix data stores in the HomogeneousMatrix
	  const double* matrixPtr;
};

}

#endif /* SCENEGRAPHNODESTEST_H_ */

/* EOF */
