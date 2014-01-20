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

}

void IdTest::testUuids() {
	std::map<Id, std::string> someMapWithIds;
	someMapWithIds.clear();

	CPPUNIT_ASSERT_EQUAL(0u , static_cast<unsigned int>(someMapWithIds.size()));

	Id someId = 0;
	someMapWithIds.insert(std::make_pair(someId, "hello"));
	CPPUNIT_ASSERT_EQUAL(1u , static_cast<unsigned int>(someMapWithIds.size()));

}

} /* namespace unitTests */

/* EOF */
