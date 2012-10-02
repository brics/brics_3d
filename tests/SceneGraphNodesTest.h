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
#include "worldModel/sceneGraph/ISceneGraphUpdateObserver.h"
#include "worldModel/sceneGraph/PathCollector.h"
#include "worldModel/sceneGraph/AttributeFinder.h"
#include "worldModel/sceneGraph/DotGraphGenerator.h"
#include "worldModel/sceneGraph/SimpleIdGenerator.h"
#include "worldModel/sceneGraph/SceneGraphFacade.h"
#include "worldModel/sceneGraph/OutdatedDataDeleter.h"
#include "worldModel/sceneGraph/PointCloudAccumulator.h"
#include "worldModel/sceneGraph/PointCloud.h"
#include "worldModel/sceneGraph/SubGraphChecker.h"
#include "core/PointCloud3D.h"
#include "core/PointCloud3DIterator.h"

#include "util/Timer.h"

namespace unitTests {

using namespace brics_3d;
using namespace Eigen;
using namespace std;
using namespace brics_3d::rsg;
using std::cout;
using std::endl;

/*
 * Example how to implement custom nodes.
 */
class CustomNode : public Node {

public:
	typedef boost::shared_ptr<CustomNode> CustomNodePtr;
	typedef boost::shared_ptr<CustomNode const> CustomNodeConstPtr;

	int someValue;

	void accept(INodeVisitor* visitor) { //optionally
		visitor->visit(this);
		if (visitor->getDirection() == INodeVisitor::upwards) {
			for(unsigned i = 0; i < getNumberOfParents(); ++i) // recursively go up the graph structure
			{
				getParent(i)->accept(visitor);
			}
		}
	}
};

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
	void visit(rsg::Transform* node){
		cout << "Transform ID = " << node->getId() << endl;
		collectedIDs.push_back(node->getId());
	};
	void visit(rsg::GeometricNode* node){
		cout << "GeometricNode ID = " << node->getId() << endl;
		collectedIDs.push_back(node->getId());
	};

	std::vector<unsigned int> collectedIDs;
};

/*
 * Example how to implement a custom observer
 */
class MyObserver : public ISceneGraphUpdateObserver {

public:
	MyObserver() {
		addNodeCounter = 0;
		addGroupCounter = 0;
		addTransformCounter = 0;
		addGeometricNodeCounter = 0;
		setNodeAttributesCounter = 0;
		setTransformCounter = 0;
		deleteNodeCounter = 0;
		addParentCounter = 0;
	}

	bool addNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, bool forceId = false) {
		addNodeCounter++;
		return true;
	}

	bool addGroup(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, bool forceId = false) {
		addGroupCounter++;
		return true;
	}

	bool addTransformNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forceId = false) {
		addTransformCounter++;
		return true;
	}

	bool addGeometricNode(unsigned int parentId, unsigned int& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forceId = false) {
		addGeometricNodeCounter++;
		return true;
	}

	bool setNodeAttributes(unsigned int id, vector<Attribute> newAttributes) {
		setNodeAttributesCounter++;
		return true;
	}

	bool setTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
		setTransformCounter++;
		return true;
	}

	bool deleteNode(unsigned int id) {
		deleteNodeCounter++;
		return true;
	}

	bool addParent(unsigned int id, unsigned int parentId) {
		addParentCounter++;
		return true;
	}

	int addNodeCounter;
	int addGroupCounter;
	int addTransformCounter;
	int addGeometricNodeCounter;
	int setNodeAttributesCounter;
	int setTransformCounter;
	int deleteNodeCounter;
	int addParentCounter;
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
	CPPUNIT_TEST( testSceneGraphFacadeTransforms );
	CPPUNIT_TEST( testPointCloud );
	CPPUNIT_TEST( testUpdateObserver );
	CPPUNIT_TEST( testDotGraphGenerator );
	CPPUNIT_TEST( testPointIterator );
	CPPUNIT_TEST( testScenePointIterator );
	CPPUNIT_TEST( testSubGraphChecker );
	CPPUNIT_TEST( testForcedIds );
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
	void testSceneGraphFacadeTransforms();
	void testPointCloud();
	void testUpdateObserver();
	void testDotGraphGenerator();
	void testPointIterator();
	void testScenePointIterator();
	void testSubGraphChecker();
	void testForcedIds();

private:
	  /// Maximum deviation for equality check of double variables
	  static const double maxTolerance = 0.00001;

	  /// Pointer to matrix data stores in the HomogeneousMatrix
	  const double* matrixPtr;
};

}

#endif /* SCENEGRAPHNODESTEST_H_ */

/* EOF */
