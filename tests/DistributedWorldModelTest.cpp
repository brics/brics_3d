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
	WorldModel* wmAlsoWithFixedId2 = 0;

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
}



}  // namespace unitTests

/* EOF */
