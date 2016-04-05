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

#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/CovarianceMatrix66.h"
#include "brics_3d/worldModel/sceneGraph/Node.h"
#include "brics_3d/worldModel/sceneGraph/Group.h"
#include "brics_3d/worldModel/sceneGraph/Transform.h"
#include "brics_3d/worldModel/sceneGraph/GeometricNode.h"
#include "brics_3d/worldModel/sceneGraph/Box.h"
#include "brics_3d/worldModel/sceneGraph/Cylinder.h"
#include "brics_3d/worldModel/sceneGraph/INodeVisitor.h"
#include "brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h"
#include "brics_3d/worldModel/sceneGraph/PathCollector.h"
#include "brics_3d/worldModel/sceneGraph/AttributeFinder.h"
#include "brics_3d/worldModel/sceneGraph/DotGraphGenerator.h"
#include "brics_3d/worldModel/sceneGraph/DotVisualizer.h"
#include "brics_3d/worldModel/sceneGraph/SimpleIdGenerator.h"
#include "brics_3d/worldModel/sceneGraph/SceneGraphFacade.h"
#include "brics_3d/worldModel/sceneGraph/OutdatedDataDeleter.h"
#include "brics_3d/worldModel/sceneGraph/PointCloudAccumulator.h"
#include "brics_3d/worldModel/sceneGraph/PointCloud.h"
#include "brics_3d/worldModel/sceneGraph/SubGraphChecker.h"
#include "brics_3d/worldModel/sceneGraph/SceneGraphToUpdatesTraverser.h"
#include "brics_3d/worldModel/sceneGraph/UncertainTransform.h"
#include "brics_3d/worldModel/sceneGraph/Connection.h"
#include "brics_3d/worldModel/sceneGraph/SemanticContextUpdateFilter.h"
#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/PointCloud3DIterator.h"

#include "brics_3d/util/Timer.h"

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
//		cout << "Node ID  = " << node->getId() << endl;
		collectedIDs.push_back(node->getId());
	};
	void visit(Group* node){
//		cout << "Group ID = " << node->getId() << endl;
		collectedIDs.push_back(node->getId());
	};
	void visit(rsg::Transform* node){
//		cout << "Transform ID = " << node->getId() << endl;
		collectedIDs.push_back(node->getId());
	};
	void visit(rsg::GeometricNode* node){
//		cout << "GeometricNode ID = " << node->getId() << endl;
		collectedIDs.push_back(node->getId());
	};
	void visit(rsg::Connection* connection){
//		cout << "GeometricNode ID = " << node->getId() << endl;
		collectedIDs.push_back(connection->getId());
	};


	std::vector<Id> collectedIDs;
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
		addUncertainTransformCounter = 0;
		addGeometricNodeCounter = 0;
		addRemoteRootNodeCounter = 0;
		addConnectionCounter = 0;
		setNodeAttributesCounter = 0;
		setTransformCounter = 0;
		setUncertainTransformCounter = 0;
		deleteNodeCounter = 0;
		addParentCounter = 0;
		removeParentCounter = 0;
	}

	bool addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forceId = false) {
		addNodeCounter++;
		return true;
	}

	bool addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forceId = false) {
		addGroupCounter++;
		return true;
	}

	bool addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forceId = false) {
		addTransformCounter++;
		return true;
	}

	bool addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forceId = false) {
		addUncertainTransformCounter++;
		return true;
	}

	bool addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forceId = false) {
		addGeometricNodeCounter++;
		return true;
	}

    bool addRemoteRootNode(Id rootId, vector<Attribute> attributes) {
    	addRemoteRootNodeCounter++;
		return true;
    }

    bool addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId = false) {
    	addConnectionCounter++;
    	return true;
    }

	bool setNodeAttributes(Id id, vector<Attribute> newAttributes, TimeStamp timeStamp = TimeStamp(0)) {
		setNodeAttributesCounter++;
		return true;
	}

	bool setTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp) {
		setTransformCounter++;
		return true;
	}

	bool setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp) {
		setUncertainTransformCounter++;
		return true;
	}

	bool deleteNode(Id id) {
		deleteNodeCounter++;
		return true;
	}

	bool addParent(Id id, Id parentId) {
		addParentCounter++;
		return true;
	}

	bool removeParent(Id id, Id parentId) {
		removeParentCounter++;
		return true;
	}

	int addNodeCounter;
	int addGroupCounter;
	int addTransformCounter;
	int addUncertainTransformCounter;
	int addGeometricNodeCounter;
	int addRemoteRootNodeCounter;
	int addConnectionCounter;
	int setNodeAttributesCounter;
	int setTransformCounter;
	int setUncertainTransformCounter;
	int deleteNodeCounter;
	int addParentCounter;
	int removeParentCounter;
};


class SceneGraphNodesTest : public CPPUNIT_NS::TestFixture {

	CPPUNIT_TEST_SUITE( SceneGraphNodesTest );
	CPPUNIT_TEST( testNode );
	CPPUNIT_TEST( testGroup );
	CPPUNIT_TEST( testTransform );
	CPPUNIT_TEST( testTemporalTransform);
	CPPUNIT_TEST( testTemporalTransformAccess);
	CPPUNIT_TEST( testUncertainTransform );
	CPPUNIT_TEST( testTemporalUncertainTransform );
	CPPUNIT_TEST( testGeometricNode );
	CPPUNIT_TEST( testConnection );
	CPPUNIT_TEST( testSimpleGraph );
	CPPUNIT_TEST( testOwnership );
	CPPUNIT_TEST( testSimpleVisitor );
	CPPUNIT_TEST( testPathCollectorVisitor );
	CPPUNIT_TEST( testTransformVisitor );
	CPPUNIT_TEST( testUncertainTransformVisitor );
	CPPUNIT_TEST( testGlobalTransformCalculation );
	CPPUNIT_TEST( testAttributeFinder );
	CPPUNIT_TEST( testOutdatedDataDeleter );
	CPPUNIT_TEST( testIdGenerator );
	CPPUNIT_TEST( testSceneGraphFacade );
	CPPUNIT_TEST( testSceneGraphFacadeTransforms );
	CPPUNIT_TEST( testSceneGraphFacadeConnections );
	CPPUNIT_TEST( testPointCloud );
	CPPUNIT_TEST( testUpdateObserver );
	CPPUNIT_TEST( testDotGraphGenerator );
	CPPUNIT_TEST( testPointIterator );
	CPPUNIT_TEST( testScenePointIterator );
	CPPUNIT_TEST( testSubGraphChecker );
	CPPUNIT_TEST( testForcedIds );
	CPPUNIT_TEST( testSceneGraphToUpdates );
	CPPUNIT_TEST( testRemoveParents );
	CPPUNIT_TEST( testNodeStorage );
	CPPUNIT_TEST( testDuplicatedInsertions );
	CPPUNIT_TEST( testTransformDurationConfig );
	CPPUNIT_TEST( testSemanticContextUpdateFilter );
	CPPUNIT_TEST( testGlobalRootNodeQueries );
	CPPUNIT_TEST_SUITE_END();

public:
	void setUp();
	void tearDown();

	void testNode();
	void testGroup();
	void testTransform();
	void testTemporalTransform();
	void testTemporalTransformAccess();
	void testUncertainTransform();
	void testTemporalUncertainTransform();
	void testGeometricNode();
	void testConnection();
	void testOwnership();
	void testSimpleGraph();
	void testSimpleVisitor();
	void testPathCollectorVisitor();
	void testTransformVisitor();
	void testUncertainTransformVisitor();
	void testGlobalTransformCalculation();
	void testAttributeFinder();
	void testOutdatedDataDeleter();
	void testIdGenerator();
	void testSceneGraphFacade();
	void testSceneGraphFacadeTransforms();
	void testSceneGraphFacadeConnections();
	void testPointCloud();
	void testUpdateObserver();
	void testDotGraphGenerator();
	void testPointIterator();
	void testScenePointIterator();
	void testSubGraphChecker();
	void testForcedIds();
	void testSceneGraphToUpdates();
	void testRemoveParents();
	void testNodeStorage();
	void testDuplicatedInsertions();
	void testTransformDurationConfig();
	void testSemanticContextUpdateFilter();
	void testGlobalRootNodeQueries();

private:
	  /// Maximum deviation for equality check of double variables
	  static const double maxTolerance = 0.00001;

	  /// Pointer to matrix data stores in the HomogeneousMatrix
	  const double* matrixPtr;
};

}

#endif /* SCENEGRAPHNODESTEST_H_ */

/* EOF */
