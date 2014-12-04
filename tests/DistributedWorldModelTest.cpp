/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2014, KU Leuven
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

#include "DistributedWorldModelTest.h"
#include "SceneGraphNodesTest.h"

namespace unitTests {

CPPUNIT_TEST_SUITE_REGISTRATION( DistributedWorldModelTest );

void DistributedWorldModelTest::setUp() {
	Logger::setMinLoglevel(Logger::LOGDEBUG);
}

void DistributedWorldModelTest::tearDown() {
	Logger::setMinLoglevel(Logger::INFO);
}

void DistributedWorldModelTest::testRootIds() {
	WorldModel* wm = 0;
	WorldModel* remoteWm = 0;

	Id fixedRootId0 = 0;
	Id fixedRootId1 = 1;
	Id fixedRootId2 = 2;
	WorldModel* wmWithFixedId0 = 0;
	WorldModel* wmWithFixedId1 = 0;
	WorldModel* wmWithFixedId2 = 0;
//	WorldModel* wmAlsoWithFixedId2 = 0;

	wm = new WorldModel();
	remoteWm = new WorldModel();
	CPPUNIT_ASSERT(wm != 0);
	CPPUNIT_ASSERT(remoteWm != 0);


	wmWithFixedId1 = new WorldModel(new UuidGenerator(fixedRootId1));
	CPPUNIT_ASSERT(wmWithFixedId1 != 0);
	CPPUNIT_ASSERT(fixedRootId1 == wmWithFixedId1->getRootNodeId());

	wmWithFixedId2 = new WorldModel(new UuidGenerator(fixedRootId2));
	CPPUNIT_ASSERT(wmWithFixedId2 != 0);
	CPPUNIT_ASSERT(fixedRootId2 == wmWithFixedId2->getRootNodeId());

	/* Feed witz NIL, expected fallback: generate a new and valid ID */
	CPPUNIT_ASSERT(fixedRootId0.isNil());
	wmWithFixedId0 = new WorldModel(new UuidGenerator(fixedRootId0));
	CPPUNIT_ASSERT(wmWithFixedId0 != 0);
	CPPUNIT_ASSERT(fixedRootId0 != wmWithFixedId0->getRootNodeId());



	Id rootId = wm->getRootNodeId();
	Id remotRootId = remoteWm->getRootNodeId();
	CPPUNIT_ASSERT(rootId != remotRootId);



	delete wm;
	delete remoteWm;
	delete wmWithFixedId0;
	delete wmWithFixedId1;
	delete wmWithFixedId2;
}

void DistributedWorldModelTest::testRemoteRootNodeCreationAndDeletion() {
	Id localRootId = 1u;
	WorldModel* wm = new WorldModel(new UuidGenerator(localRootId));
	CPPUNIT_ASSERT(!wm->getRootNodeId().isNil());
	CPPUNIT_ASSERT(wm->getRootNodeId() == localRootId);
	vector<Attribute> dummyAttributes;

	// preconditions
	vector<Id> remoteRootIds;
	vector<Attribute> attributes;
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(remoteRootIds.size()));

	/* check prevention self inclusion */
	CPPUNIT_ASSERT(!wm->scene.addRemoteRootNode(wm->getRootNodeId(),attributes));
	remoteRootIds.clear();
	CPPUNIT_ASSERT(wm->scene.getRemoteRootNodes(remoteRootIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(remoteRootIds.size()));

	/* valid Ids*/
	Id remotRootId1 = 2u;
	Id remotRootId2 = 3u;

	CPPUNIT_ASSERT(wm->scene.addRemoteRootNode(remotRootId1, attributes));
	remoteRootIds.clear();
	CPPUNIT_ASSERT(wm->scene.getRemoteRootNodes(remoteRootIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(remoteRootIds.size()));
	CPPUNIT_ASSERT(remoteRootIds[0] ==  remotRootId1);

	CPPUNIT_ASSERT(wm->scene.addRemoteRootNode(remotRootId2, attributes));
	remoteRootIds.clear();
	CPPUNIT_ASSERT(wm->scene.getRemoteRootNodes(remoteRootIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(remoteRootIds.size()));
	CPPUNIT_ASSERT(remoteRootIds[0] ==  remotRootId1);
	CPPUNIT_ASSERT(remoteRootIds[1] ==  remotRootId2);

	// chck existence
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(remotRootId1, dummyAttributes));
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(remotRootId2, dummyAttributes));

	/* Remove first one  */
	CPPUNIT_ASSERT(wm->scene.deleteNode(remotRootId1));
	remoteRootIds.clear();
	CPPUNIT_ASSERT(wm->scene.getRemoteRootNodes(remoteRootIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(remoteRootIds.size()));
	CPPUNIT_ASSERT(remoteRootIds[0] ==  remotRootId2);

	CPPUNIT_ASSERT(!wm->scene.getNodeAttributes(remotRootId1, dummyAttributes));
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(remotRootId2, dummyAttributes));

	CPPUNIT_ASSERT(!wm->scene.deleteNode(remotRootId1));
	remoteRootIds.clear();
	CPPUNIT_ASSERT(wm->scene.getRemoteRootNodes(remoteRootIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(remoteRootIds.size()));
	CPPUNIT_ASSERT(remoteRootIds[0] ==  remotRootId2);

	/*
	 * add remot node to
	 */
	CPPUNIT_ASSERT(wm->scene.addParent(remotRootId2, localRootId));

	CPPUNIT_ASSERT(!wm->scene.getNodeAttributes(remotRootId1, dummyAttributes));
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(remotRootId2, dummyAttributes));

	/*
	 * remove last one
	 */
	CPPUNIT_ASSERT(wm->scene.deleteNode(remotRootId2));
	remoteRootIds.clear();
	CPPUNIT_ASSERT(wm->scene.getRemoteRootNodes(remoteRootIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(remoteRootIds.size()));


	CPPUNIT_ASSERT(!wm->scene.getNodeAttributes(remotRootId1, dummyAttributes));
	CPPUNIT_ASSERT(!wm->scene.getNodeAttributes(remotRootId2, dummyAttributes));

	delete wm;
}

void DistributedWorldModelTest::testRemoteRootNodes() {

	WorldModel* wm = 0;
	WorldModel* remoteWm = 0;
	wm = new WorldModel();
	remoteWm = new WorldModel();
	CPPUNIT_ASSERT(wm->getRootNodeId() != remoteWm->getRootNodeId());
	remoteWm->scene.setCallObserversEvenIfErrorsOccurred(false); //has to be false such thet the counter is not invoked on an errournous insertion

	MyObserver wmNodeCounter;
	MyObserver remoteWmNodeCounter;

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	wm->scene.attachUpdateObserver(&wmNodeCounter);
	remoteWm->scene.attachUpdateObserver(&remoteWmNodeCounter);
	SceneGraphToUpdatesTraverser wmToRemoteWm(&remoteWm->scene);


	Id groupId;
	vector<Attribute> attributes;
	wm->scene.addGroup(wm->getRootNodeId(), groupId, attributes);
	wm->scene.executeGraphTraverser(&wmToRemoteWm, wm->getRootNodeId());	// Push complete graph tor remote wm

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter); //cannot be ther because o missing root ID
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	/* make first root ID avaoulable to remotWm*/
	vector<Attribute> rootNodeAttributes;
	wm->scene.getNodeAttributes(wm->getRootNodeId(), rootNodeAttributes);
	remoteWm->scene.addRemoteRootNode(wm->getRootNodeId(), rootNodeAttributes);
	wm->scene.addGroup(wm->getRootNodeId(), groupId, attributes);
	wm->scene.executeGraphTraverser(&wmToRemoteWm, wm->getRootNodeId());	// Push complete graph tor remote wm

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter); //now the update should be there
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);


	delete wm;
	delete remoteWm;

}

void DistributedWorldModelTest::testRemoteRootNodeQueries() {
	WorldModel* wm = new WorldModel;
	WorldModel* remoteWm = new WorldModel;
	vector<Attribute> attributes;
	vector<Attribute> resultAttributes;
	vector<Id> resultIds;

	/*
	 * setup wm
	 */
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "root1"));
	wm->scene.setNodeAttributes(wm->getRootNodeId(), attributes);

	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "tf_1"));
	rsg::Id tf1Id;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); 						// Translation coefficients
	wm->scene.addTransformNode(wm->getRootNodeId(), tf1Id, attributes, transform123, wm->now());

	/*
	 * setup remoteWm
	 */
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "root2"));
	attributes.push_back(rsg::Attribute("info", "a_remote_world_model"));
	remoteWm->scene.setNodeAttributes(remoteWm->getRootNodeId(), attributes);

	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "tf_2"));
	rsg::Id tf2Id;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminus123(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             -1,-2,-3)); 						// Translation coefficients
	remoteWm->scene.addTransformNode(remoteWm->getRootNodeId(), tf2Id, attributes, transformminus123, remoteWm->now());


	/*
	 * Both WorldModel agend are not connected, thus we have a forest. Queries ere not possible beteen them
	 *
	 *     wm         remoteWM
	 *     |             |
	 *    root1        root2
	 *     |             |
	 *	  tf_1          tf_2
	 */
	/* Queries on wm */
	resultAttributes.clear();
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultAttributes.size()));

	resultAttributes.clear();
	CPPUNIT_ASSERT(!wm->scene.getNodeAttributes(remoteWm->getRootNodeId(), resultAttributes)); // try to read the other root node

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult1;
	CPPUNIT_ASSERT(wm->scene.getTransformForNode(tf1Id, wm->getRootNodeId(), wm->now(), transformminusResult1));

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult2;
	CPPUNIT_ASSERT(!wm->scene.getTransformForNode(tf1Id, tf2Id, wm->now(), transformminusResult2));

	/* Queries on remoteWm */
	resultAttributes.clear();
	CPPUNIT_ASSERT(remoteWm->scene.getNodeAttributes(remoteWm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultAttributes.size()));

	resultAttributes.clear();
	CPPUNIT_ASSERT(!remoteWm->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes)); // try to read the other root node

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult3;
	CPPUNIT_ASSERT(remoteWm->scene.getTransformForNode(tf2Id, remoteWm->getRootNodeId(), remoteWm->now(), transformminusResult3));

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult4;
	CPPUNIT_ASSERT(!remoteWm->scene.getTransformForNode(tf1Id, tf2Id, remoteWm->now(), transformminusResult4));

	/*
	 * Now we add one root node as a reomoreRoot to the other one.
	 * Note in wm the local and remots nodes arwe not yet connected
	 * "intra queries" are not possible.
	 *
	 *
	 *     wm  (remotes)   remoteWM
	 *     +.....+            +
	 *     |     |            |
	 *    root1  root2      root2
	 *     |                  |
	 *	  tf_1              rf_2
	 */
	resultAttributes.clear();
	CPPUNIT_ASSERT(remoteWm->scene.getNodeAttributes(remoteWm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT(wm->scene.addRemoteRootNode(remoteWm->getRootNodeId(), resultAttributes)); // Of course, we have to provide all relevant data including the attributes.
	CPPUNIT_ASSERT(!wm->scene.addRemoteRootNode(remoteWm->getRootNodeId(), resultAttributes)); // We can't create dublications

	/*
	 * Query again: now the retrival of the attributes of the remot node should work
	 */

	/* Queries on wm */
	resultAttributes.clear();
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultAttributes.size()));

	resultAttributes.clear();
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(remoteWm->getRootNodeId(), resultAttributes)); // try to read the other root node
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultAttributes.size()));

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult5;
	CPPUNIT_ASSERT(wm->scene.getTransformForNode(tf1Id, wm->getRootNodeId(), wm->now(), transformminusResult5));

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult6;
	CPPUNIT_ASSERT(!wm->scene.getTransformForNode(tf1Id, tf2Id, wm->now(), transformminusResult6));

	/* Get attribute should not  work yet, as a treveral works in the local scope. */
	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "root2"));
	CPPUNIT_ASSERT(wm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultIds.size()));

	/*
	 * Now we also connect the remote root wuth the local roofd node by putting the remote
	 * on as subgraph into the local one.
	 *
	 * "intra queries" are now possible.
	 *
	 *
	 *       wm  (remotes)   remoteWM
	 *       +.....+            +
	 *       |     |            |
	 *      root1  |          root2
	 *       |     |            |
	 *   +---+---+ |            |
	 *   |       | |            |
	 *	tf_1    root2          rf_2
	 */
	CPPUNIT_ASSERT(wm->scene.addParent(remoteWm->getRootNodeId(), wm->getRootNodeId()));

	/*
	 * Query again: now the retrival of the attributes of the remot node and
	 * traverals should work
	 */

	/* Queries on wm */
	resultAttributes.clear();
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultAttributes.size()));

	resultAttributes.clear();
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(remoteWm->getRootNodeId(), resultAttributes)); // try to read the other root node
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultAttributes.size()));

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult7;
	CPPUNIT_ASSERT(wm->scene.getTransformForNode(tf1Id, wm->getRootNodeId(), wm->now(), transformminusResult7));

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult8;
	CPPUNIT_ASSERT(!wm->scene.getTransformForNode(tf1Id, tf2Id, wm->now(), transformminusResult8));

	/* Get attribute should not  work yet, as a treveral works in the local scope. */
	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "root2"));
	CPPUNIT_ASSERT(wm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT(resultIds[0] == remoteWm->getRootNodeId());

	/*
	 * To make a pose query possible we have to pull all data from the remote world model
	 * rleative to the remote rood node. To do so we trigger a full traveral of the first one
	 *
	 *       wm  (remotes)   remoteWM
	 *       +.....+            +
	 *       |     |            |
	 *      root1  |          root2
	 *       |     |            |
	 *   +---+---+ |            |
	 *   |       | |            |
	 *	tf_1    root2          tf_2
	 *	          |
	 *	         tf2
	 */
	SceneGraphToUpdatesTraverser wmFromRemoteWm(&wm->scene);
	remoteWm->scene.executeGraphTraverser(&wmFromRemoteWm, remoteWm->getRootNodeId());	// Pull complete graph from remoteWm to wm

	/*
	 * Query again: now the retrival of the attributes of the remote node, the
	 * traverals and pose queries between the data should work
	 */

	/* Queries on wm */
	resultAttributes.clear();
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultAttributes.size()));

	resultAttributes.clear();
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(remoteWm->getRootNodeId(), resultAttributes)); // try to read the other root node
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultAttributes.size()));

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult9;
	CPPUNIT_ASSERT(wm->scene.getTransformForNode(tf1Id, wm->getRootNodeId(), wm->now(), transformminusResult9));

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult10;
	CPPUNIT_ASSERT(wm->scene.getTransformForNode(tf1Id, tf2Id, wm->now(), transformminusResult10));

	//check data
	const double* matrix1 = transformminusResult10->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2, matrix1[matrixEntry::x], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4, matrix1[matrixEntry::y], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6, matrix1[matrixEntry::z], maxTolerance);

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult11;
	CPPUNIT_ASSERT(wm->scene.getTransformForNode(tf2Id, tf1Id, wm->now(), transformminusResult11));

	//check data
	const double* matrix2 = transformminusResult11->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-2, matrix2[matrixEntry::x], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-4, matrix2[matrixEntry::y], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(-6, matrix2[matrixEntry::z], maxTolerance);

	/* Get attribute should not  work yet, as a treveral works in the local scope. */
	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "root2"));
	CPPUNIT_ASSERT(wm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT(resultIds[0] == remoteWm->getRootNodeId());

	/*
	 * Now the other way around:
	 * 1.) add remoteNode in cluding retrival of current attributes
	 * 2.) hook it into the local graph
	 * 3.) perform a complete sync
	 */
	// 1.)
	resultAttributes.clear();
	CPPUNIT_ASSERT(wm->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT(remoteWm->scene.addRemoteRootNode(wm->getRootNodeId(), resultAttributes));
	// 2.)
	CPPUNIT_ASSERT(remoteWm->scene.addParent(wm->getRootNodeId(), remoteWm->getRootNodeId()));
	// 3.)
	SceneGraphToUpdatesTraverser remoteWmFromWm(&remoteWm->scene);
	wm->scene.executeGraphTraverser(&remoteWmFromWm, wm->getRootNodeId());


	resultAttributes.clear();
	CPPUNIT_ASSERT(remoteWm->scene.getNodeAttributes(remoteWm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultAttributes.size()));

	resultAttributes.clear();
	CPPUNIT_ASSERT(remoteWm->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultAttributes.size()));

	resultAttributes.clear();
	CPPUNIT_ASSERT(remoteWm->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes)); // try to read the other root node

	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "root1"));
	CPPUNIT_ASSERT(remoteWm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT(resultIds[0] == wm->getRootNodeId());

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult12;
	CPPUNIT_ASSERT(remoteWm->scene.getTransformForNode(tf2Id, remoteWm->getRootNodeId(), remoteWm->now(), transformminusResult12));

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformminusResult13;
	CPPUNIT_ASSERT(remoteWm->scene.getTransformForNode(tf1Id, tf2Id, remoteWm->now(), transformminusResult13));

	//check data
	const double* matrix3 = transformminusResult13->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(2, matrix3[matrixEntry::x], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4, matrix3[matrixEntry::y], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6, matrix3[matrixEntry::z], maxTolerance);

	/*
	 * Further testst
	 */
	CPPUNIT_ASSERT(!wm->scene.addParent(wm->getRootNodeId(), wm->getRootNodeId())); // prevent loops

	delete wm;
	delete remoteWm;
}

void DistributedWorldModelTest::testDirectUpdateObserver() {

	WorldModel* wm = new WorldModel();
	WorldModel* remoteWm = new WorldModel();
	CPPUNIT_ASSERT(wm->getRootNodeId() != remoteWm->getRootNodeId());
	remoteWm->scene.setCallObserversEvenIfErrorsOccurred(false); //has to be false such thet the counter is not invoked on an errournous insertion

	vector<Attribute> attributes;
	vector<Attribute> resultAttributes;
	vector<Id> resultIds;

	MyObserver wmNodeCounter;
	MyObserver remoteWmNodeCounter;

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	/*
	 * Various observer attachments
	 */
	CPPUNIT_ASSERT(wm->scene.attachUpdateObserver(&wmNodeCounter));
//	CPPUNIT_ASSERT(wm->scene.attachUpdateObserver(&(remoteWm->scene))); // this is not possible
	// we directly let one WM observe the other one
	UpdatesToSceneGraphListener wmToRemoteWmListener;
	CPPUNIT_ASSERT(wm->scene.attachUpdateObserver(&wmToRemoteWmListener));
	wmToRemoteWmListener.attachSceneGraph(&remoteWm->scene);
	CPPUNIT_ASSERT(remoteWm->scene.attachUpdateObserver(&remoteWmNodeCounter));

	Id groupId;
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "group1"));
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	wm->scene.addGroup(wm->getRootNodeId(), groupId, attributes);

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter); // update is not available to the other wm as the root node doe not exist
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	CPPUNIT_ASSERT(remoteWm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultIds.size()));

	/*
	 * Now we add the remote node but we keep it unconnected
	 */
	attributes.clear(); // here we ommit the attributes
	remoteWm->scene.addRemoteRootNode(wm->getRootNodeId(), attributes);


	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "group2"));
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	wm->scene.addGroup(wm->getRootNodeId(), groupId, attributes);

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter); // update should be there
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	CPPUNIT_ASSERT(remoteWm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(0u, static_cast<unsigned int>(resultIds.size())); // should still fail as there is no connection between the graphs

	/*
	 * Finally connect the graphs within the remote root node
	 */
	CPPUNIT_ASSERT(remoteWm->scene.addParent(wm->getRootNodeId(), remoteWm->getRootNodeId()));

	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "group3"));
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	wm->scene.addGroup(wm->getRootNodeId(), groupId, attributes);

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addGroupCounter); // update should be there
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	CPPUNIT_ASSERT(remoteWm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultIds.size())); // now the updates since the addRemoteRoot should be available

	/*
	 * Trigger a full traversal on the first one to get them fully synchronized.
	 * Can be used to solve the late joins.
	 */
	SceneGraphToUpdatesTraverser wmToRemotWm(&wmToRemoteWmListener);
	wm->scene.executeGraphTraverser(&wmToRemotWm, wm->getRootNodeId());

	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	CPPUNIT_ASSERT(wm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(3u, static_cast<unsigned int>(resultIds.size()));

	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	CPPUNIT_ASSERT(remoteWm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(3u, static_cast<unsigned int>(resultIds.size())); // now the last update should be available

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addGroupCounter); // update should be there
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	/*
	 * Now both are synronized. Further sucessive updates should be passed automatically
	 */

	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "group4"));
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	wm->scene.addGroup(wm->getRootNodeId(), groupId, attributes);

	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	CPPUNIT_ASSERT(wm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(4u, static_cast<unsigned int>(resultIds.size()));

	resultIds.clear();
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "some_common_tag"));
	CPPUNIT_ASSERT(remoteWm->scene.getNodes(attributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(4u, static_cast<unsigned int>(resultIds.size())); // now the last update should be available

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGroupCounter); // update should be there
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	/*
	 * Test obesrvability of addRootNodes on obesrves wm
	 */
	Id dummyRootId = 42;
	CPPUNIT_ASSERT(wm->scene.addRemoteRootNode(dummyRootId, attributes));

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGroupCounter); // update should be there
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	delete wm;
	delete remoteWm;
}

void DistributedWorldModelTest::testManualSelfAnnouncement() {

	/*
	 * Essntially a manual self announcement is not possible, beacause
	 * wm->scene.addRemoteRootNode(wm->getRootNodeId(), attributes) i.e.
	 * adding the own root node as Root node is an error, wich is not
	 * beeing propagated when using the policy:
	 * wm->scene.setCallObserversEvenIfErrorsOccurred(false);
	 * This policy is required for a distibuted setting including
	 * loop back connections
	 */

	WorldModel* wm = new WorldModel();
	WorldModel* remoteWm = new WorldModel();
	CPPUNIT_ASSERT(wm->getRootNodeId() != remoteWm->getRootNodeId());
	wm->scene.setCallObserversEvenIfErrorsOccurred(true);
	remoteWm->scene.setCallObserversEvenIfErrorsOccurred(true); //has to be false such thet the counter is not invoked on an errournous insertion

	vector<Attribute> attributes;
	vector<Attribute> resultAttributes;
	vector<Id> resultIds;

	MyObserver wmNodeCounter;
	MyObserver remoteWmNodeCounter;

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	/*
	 * Various observer attachments
	 */
	CPPUNIT_ASSERT(wm->scene.attachUpdateObserver(&wmNodeCounter));
	UpdatesToSceneGraphListener wmToRemoteWmListener;
	CPPUNIT_ASSERT(wm->scene.attachUpdateObserver(&wmToRemoteWmListener));
	wmToRemoteWmListener.attachSceneGraph(&remoteWm->scene);
	CPPUNIT_ASSERT(remoteWm->scene.attachUpdateObserver(&remoteWmNodeCounter));


	/*
	 * Test manual self annoucement
	 */
	wm->scene.setCallObserversEvenIfErrorsOccurred(false);
	remoteWm->scene.setCallObserversEvenIfErrorsOccurred(false);
	CPPUNIT_ASSERT(!wm->scene.addRemoteRootNode(wm->getRootNodeId(), attributes));

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT(!remoteWm->scene.getNodeAttributes(wm->getRootNodeId(), attributes));

	CPPUNIT_ASSERT(!wm->scene.addRemoteRootNode(wm->getRootNodeId(), attributes));

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	/*
	 * again with the non-dafault policy
	 */
	wm->scene.setCallObserversEvenIfErrorsOccurred(true);
	remoteWm->scene.setCallObserversEvenIfErrorsOccurred(true);
	CPPUNIT_ASSERT(!wm->scene.addRemoteRootNode(wm->getRootNodeId(), attributes));

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT(remoteWm->scene.getNodeAttributes(wm->getRootNodeId(), attributes));

	CPPUNIT_ASSERT(!wm->scene.addRemoteRootNode(wm->getRootNodeId(), attributes));

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT(remoteWm->scene.getNodeAttributes(wm->getRootNodeId(), attributes));

	delete wm;
	delete remoteWm;
}

void DistributedWorldModelTest::testSelfAnnouncement() {
	WorldModel* wm = new WorldModel();
	WorldModel* remoteWm = new WorldModel();
	CPPUNIT_ASSERT(wm->getRootNodeId() != remoteWm->getRootNodeId());

	vector<Attribute> attributes;
	vector<Attribute> resultAttributes;
	vector<Id> resultIds;

	MyObserver wmNodeCounter;
	MyObserver remoteWmNodeCounter;

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	/*
	 * Various observer attachments
	 */
	CPPUNIT_ASSERT(wm->scene.attachUpdateObserver(&wmNodeCounter));
	UpdatesToSceneGraphListener wmToRemoteWmListener;
	CPPUNIT_ASSERT(wm->scene.attachUpdateObserver(&wmToRemoteWmListener));
	wmToRemoteWmListener.attachSceneGraph(&remoteWm->scene);
	CPPUNIT_ASSERT(remoteWm->scene.attachUpdateObserver(&remoteWmNodeCounter));


	/*
	 * Test self annoucement
	 */
	wm->scene.setCallObserversEvenIfErrorsOccurred(false);
	remoteWm->scene.setCallObserversEvenIfErrorsOccurred(true); // true just for the testability

	CPPUNIT_ASSERT(wm->scene.advertiseRootNode());

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT(wm->scene.advertiseRootNode());

	CPPUNIT_ASSERT(!wm->scene.addRemoteRootNode(wm->getRootNodeId(), attributes));

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);

	CPPUNIT_ASSERT(remoteWm->scene.getNodeAttributes(wm->getRootNodeId(), attributes));
}

}  // namespace unitTests

/* EOF */
