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
#include "IdTest.h"

namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( IdTest );

void IdTest::setUp() {

}

void IdTest::tearDown() {

}

void IdTest::testIds() {
	Id rootId;
	Id node1Id;
	Id node2Id;
	Id node2IdCopy;

	rootId = 0;
	node1Id = 1;
	node2Id = 2;
	node2IdCopy = node2Id;

	/* test (in-) equality relaitons */
	CPPUNIT_ASSERT(rootId != node1Id);
	CPPUNIT_ASSERT(node1Id != node2Id);
	CPPUNIT_ASSERT(node2Id == node2IdCopy);

	CPPUNIT_ASSERT(node1Id < node2Id);
	CPPUNIT_ASSERT(node1Id <= node2Id);
	CPPUNIT_ASSERT(node2Id > node1Id);
	CPPUNIT_ASSERT(node2Id >= node1Id);

	/* Test in a map */
	std::map<Id, std::string> someMapWithIds;
	someMapWithIds.clear();

	CPPUNIT_ASSERT_EQUAL(0u , static_cast<unsigned int>(someMapWithIds.size()));

	Id someId = 0;
	someMapWithIds.insert(std::make_pair(someId, "hello"));
	CPPUNIT_ASSERT_EQUAL(1u , static_cast<unsigned int>(someMapWithIds.size()));


}

void IdTest::testUuids() {
	Uuid rootId;
	Uuid node1Id;
	Uuid node2Id;
	Uuid node2IdCopy;

	rootId = 0;
	node1Id = 1;
	node2Id = 2;
	node2IdCopy = node2Id;

	/* test (in-) equality relations */
	CPPUNIT_ASSERT(rootId != node1Id);
	CPPUNIT_ASSERT(node1Id != node2Id);
	CPPUNIT_ASSERT(node2Id == node2IdCopy);

	CPPUNIT_ASSERT(node1Id < node2Id);
	CPPUNIT_ASSERT(node1Id <= node2Id);
	CPPUNIT_ASSERT(node2Id > node1Id);
	CPPUNIT_ASSERT(node2Id >= node1Id);


	/* Test in a map */
	std::map<Uuid, std::string> someMapWithIds;
	someMapWithIds.clear();

	CPPUNIT_ASSERT_EQUAL(0u , static_cast<unsigned int>(someMapWithIds.size()));

	Uuid someId; // = 0
	someId = 0;
	someMapWithIds.insert(std::make_pair(someId, "hello"));
	CPPUNIT_ASSERT_EQUAL(1u , static_cast<unsigned int>(someMapWithIds.size()));

	/* Test UUID specifics */
	Uuid anotherId;
	CPPUNIT_ASSERT(anotherId.isNil());
	anotherId = 1;
	CPPUNIT_ASSERT(!anotherId.isNil());
	Uuid nullId = 0;
	CPPUNIT_ASSERT(nullId.isNil());

	Uuid anotherIdCopy (anotherId);
	CPPUNIT_ASSERT(anotherIdCopy == anotherIdCopy);

	/* Test constness */
	const Uuid node3Id;
	CPPUNIT_ASSERT(node3Id.isNil());
	CPPUNIT_ASSERT(node1Id != node3Id);
	CPPUNIT_ASSERT(node3Id != node1Id);
	CPPUNIT_ASSERT(!(node1Id == node3Id));
	CPPUNIT_ASSERT(!(node3Id == node1Id));

	CPPUNIT_ASSERT(!(node1Id < node3Id));
	CPPUNIT_ASSERT(!(node1Id <= node3Id));
	CPPUNIT_ASSERT(node2Id > node3Id);
	CPPUNIT_ASSERT(node2Id >= node3Id);

	/* test swapping */
	Uuid node4Id = 4;
	Uuid node5Id = 5;
//	node4Id = 4;
//	node5Id = 5;
	Uuid node4IdCopy(node4Id);
	Uuid node5IdCopy(node5Id);

	CPPUNIT_ASSERT(node4IdCopy == node4Id);
	CPPUNIT_ASSERT(node5IdCopy == node5Id);
	CPPUNIT_ASSERT(node4IdCopy < node5IdCopy);
	node4IdCopy.swap(node5IdCopy);
	CPPUNIT_ASSERT(node4IdCopy == node5Id);
	CPPUNIT_ASSERT(node5IdCopy == node4Id);
	CPPUNIT_ASSERT(node4IdCopy > node5IdCopy);

	LOG(DEBUG) << "rootId as UUID = " << rootId.toString();
	LOG(DEBUG) << "node1Id as UUID = " << node1Id.toString();
	LOG(DEBUG) << "node2Id as UUID = " << node2Id.toString();
	LOG(DEBUG) << "node2IdCopy as UUID = " << node2IdCopy.toString();
	LOG(DEBUG) << "node3Id as UUID = " << node3Id.toString();
	LOG(DEBUG) << "node4Id as UUID = " << node4Id;
	LOG(DEBUG) << "node5Id as UUID = " << node5Id;

	Uuid node6Id;
	unsigned int idValue = 513;
	node6Id = idValue;
	LOG(DEBUG) << "node6Id as UUID = " << node6Id;
	unsigned int node6IdAsInt = uuidToUnsignedInt(node6Id);
	CPPUNIT_ASSERT_EQUAL(idValue, node6IdAsInt);
}

void IdTest::testUuidConstructor() {

	std::vector<brics_3d::rsg::Id> inputDataIds;
	unsigned int inputPointCloudId;

	inputDataIds.clear();
	inputPointCloudId = 41;
	inputDataIds.push_back(inputPointCloudId);

	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(inputDataIds.size()));
}



void IdTest::testUuidGenerator() {
	UuidGenerator idGenerator;

	Uuid rootID = idGenerator.getRootId();
	Uuid nullId = 0;
	CPPUNIT_ASSERT(rootID != nullId);

	Uuid id1 = idGenerator.getNextValidId();
	Uuid id2 = idGenerator.getNextValidId();
	Uuid id3 = idGenerator.getNextValidId();

	LOG(DEBUG) << "id1 as UUID = " << id1;
	CPPUNIT_ASSERT(id1 != id2);
	CPPUNIT_ASSERT(id1 != id3);
	CPPUNIT_ASSERT(id2 != id1);
	CPPUNIT_ASSERT(id2 != id3);
	CPPUNIT_ASSERT(id3 != id1);
	CPPUNIT_ASSERT(id3 != id2);

	/* Test preset IDs */
	Uuid rootId42 = 42;
	UuidGenerator idGenerator2(rootId42);
	CPPUNIT_ASSERT(rootId42 == idGenerator2.getRootId());

	/* prevent invalid NIL ID */
	UuidGenerator idGenerator3(nullId);
	CPPUNIT_ASSERT(nullId != idGenerator3.getRootId());

}

void IdTest::testIdGenerator() {
	UuidGenerator idGenerator;

	Id rootID = idGenerator.getRootId();
	Id nullId = 0;
	CPPUNIT_ASSERT(rootID != nullId);

	Id id1 = idGenerator.getNextValidId();
	Id id2 = idGenerator.getNextValidId();
	Id id3 = idGenerator.getNextValidId();

	LOG(DEBUG) << "id1 as UUID = " << id1;
	CPPUNIT_ASSERT(id1 != id2);
	CPPUNIT_ASSERT(id1 != id3);
	CPPUNIT_ASSERT(id2 != id1);
	CPPUNIT_ASSERT(id2 != id3);
	CPPUNIT_ASSERT(id3 != id1);
	CPPUNIT_ASSERT(id3 != id2);

	/* Test preset IDs */
	Id rootId42 = 42;
	UuidGenerator idGenerator2(rootId42);
	CPPUNIT_ASSERT(rootId42 == idGenerator2.getRootId());

	/* prevent invalid NIL ID */
	UuidGenerator idGenerator3(nullId);
	CPPUNIT_ASSERT(nullId != idGenerator3.getRootId());
}

} /* namespace unitTests */

/* EOF */
