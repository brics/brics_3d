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

}

void DistributedWorldModelTest::tearDown() {

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


}  // namespace unitTests

/* EOF */
