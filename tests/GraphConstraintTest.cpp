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


#include "GraphConstraintTest.h"
#include "SceneGraphNodesTest.h" // for the observer counter
#include "brics_3d/core/Logger.h"
#include "brics_3d/worldModel/sceneGraph/GraphConstraint.h"

using namespace brics_3d;

namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( GraphConstraintTest );



void GraphConstraintTest::setUp() {
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
}

void GraphConstraintTest::tearDown() {
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::WARNING);
}

void GraphConstraintTest::testParser() {

	GraphConstraint c0;
	CPPUNIT_ASSERT(!c0.validate());
	CPPUNIT_ASSERT(!c0.parse("")); // empty constraint

	string policy1 = "receive no Meshes";
	string policy1_invalid = "no Meshes";
	string policy2 = "send only Transforms with dist < 2 m from me";
	string policy2_invalid = "invalid value";
	string policy3 = "receive only PointClouds with dist > 2 m from dc0829e9-71af-459e-bc4c-26b20dbd06fa";
	string policy4 = "send no Connections from context osm";
	string policy5 = "receive no Groups contained in dc0829e9-71af-459e-bc4c-26b20dbd06fa";
	string policy6 = "send no Transforms with freq > 1 Hz";
	string policy7 = "send only PointClouds with lod < 100";
	string policy7_invalid = "send no Transforms with freq > 1Hz";

	GraphConstraint c1;
	CPPUNIT_ASSERT(c1.parse(policy1));
	CPPUNIT_ASSERT(!c1.parse(policy1_invalid));

	GraphConstraint c2;
	CPPUNIT_ASSERT(c2.parse(policy2));
	CPPUNIT_ASSERT(!c2.parse(policy2_invalid));

	GraphConstraint c3;
	CPPUNIT_ASSERT(c3.parse(policy3));

	GraphConstraint c4;
	CPPUNIT_ASSERT(c4.parse(policy4));

	GraphConstraint c5;
	CPPUNIT_ASSERT(c5.parse(policy5));

	GraphConstraint c6;
	CPPUNIT_ASSERT(c6.parse(policy6));

	GraphConstraint c7;
	CPPUNIT_ASSERT(c7.parse(policy7));
	CPPUNIT_ASSERT(!c7.parse(policy7_invalid));

}

}  // namespace unitTests


/* EOF */
