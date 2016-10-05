/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2016, KU Leuven
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
#include "brics_3d/worldModel/sceneGraph/GraphConstraintUpdateFilter.h"

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
	string policy8_invalid  = "send no Atoms contained with lod > 100";

	GraphConstraint c1;
	CPPUNIT_ASSERT(c1.parse(policy1));
	CPPUNIT_ASSERT(!c1.parse(policy1_invalid));

	GraphConstraint c2;
	CPPUNIT_ASSERT(c2.parse(policy2));
	CPPUNIT_ASSERT(c2.isMe);
	CPPUNIT_ASSERT(!c2.parse(policy2_invalid));
	CPPUNIT_ASSERT(!c2.isMe);
	CPPUNIT_ASSERT(c2.parse(policy3));
	CPPUNIT_ASSERT(!c2.isMe);

	GraphConstraint c3;
	CPPUNIT_ASSERT(c3.parse(policy3));
	CPPUNIT_ASSERT(!c3.isMe);

	GraphConstraint c4;
	CPPUNIT_ASSERT(c4.parse(policy4));

	GraphConstraint c5;
	CPPUNIT_ASSERT(c5.parse(policy5));

	GraphConstraint c6;
	CPPUNIT_ASSERT(c6.parse(policy6));

	GraphConstraint c7;
	CPPUNIT_ASSERT(c7.parse(policy7));
	CPPUNIT_ASSERT(!c7.parse(policy7_invalid));

	GraphConstraint c8;
	CPPUNIT_ASSERT(!c8.parse(policy8_invalid));

}

void GraphConstraintTest::testNoConstraints() {

	WorldModel* wm = new WorldModel();
	MyObserver wmNodeCounter;
	GraphConstraintUpdateFilter filter(wm);
	wm->scene.attachUpdateObserver(&filter);
	filter.attachUpdateObserver(&wmNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	delete wm;
}

void GraphConstraintTest::testSimpleConstraints() {

	WorldModel* wm = new WorldModel();
	MyObserver wmNodeCounter;
	GraphConstraintUpdateFilter filter(wm);
	wm->scene.attachUpdateObserver(&filter);
	filter.attachUpdateObserver(&wmNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);


	GraphConstraint c1;
	CPPUNIT_ASSERT(c1.parse("send no Atoms"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);	//never blocked
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Nodes"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send only Nodes"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Groups"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send only Groups"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Transforms"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(9, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no GeometricNodes"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(12, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Connections"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(15, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no PointClouds"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(18, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(9, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(10, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(9, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Boxes"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(21, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(10, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(12, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(10, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.removeParentCounter);

	delete wm;

}

void GraphConstraintTest::testInvalidConstraints() {

	WorldModel* wm = new WorldModel();
	MyObserver wmNodeCounter;
	GraphConstraintUpdateFilter filter(wm);
	wm->scene.attachUpdateObserver(&filter);
	filter.attachUpdateObserver(&wmNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);


	GraphConstraint c1;
	CPPUNIT_ASSERT(!c1.parse("send only INVALID STUFF"));
	CPPUNIT_ASSERT(!c1.validate());
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	delete wm;
}

void GraphConstraintTest::testSemanticContextConstraints() {

	WorldModel* wm = new WorldModel();
	MyObserver wmNodeCounter;
	GraphConstraintUpdateFilter filter(wm);
	wm->scene.attachUpdateObserver(&filter);
	filter.attachUpdateObserver(&wmNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);


	GraphConstraint c1;
	CPPUNIT_ASSERT(c1.parse("send only Atoms"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Atoms from context osm"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send only Atoms from context osm"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.deleteNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Transforms from context tf"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(9, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter); // != uncertain transform
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Nodes from context gis"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(11, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addTransformCounter); // != uncertain transform
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send only Nodes from context gis"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(12, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addTransformCounter); // != uncertain transform
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Nodes from context "));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(15, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addTransformCounter); // != uncertain transform
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.removeParentCounter);

	delete wm;
}

void GraphConstraintTest::testDistanceConstraints() {
	WorldModel* wm = new WorldModel();
	MyObserver wmNodeCounter;
	GraphConstraintUpdateFilter filter(wm);
	wm->scene.attachUpdateObserver(&filter);
	filter.attachUpdateObserver(&wmNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);


	GraphConstraint c1;
	CPPUNIT_ASSERT(c1.parse("send only Atoms with dist < 10 m from me"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send only Atoms with dist > 10 m from me"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	/*
	 * NOTE:
	 *  * transform123 => dist = ~3.7
	 *  * transform234 => dist = ~5.7
	 *  * transform456 => dist = ~8.7
	 */

	CPPUNIT_ASSERT(c1.parse("send only Atoms with dist < 5 m from me"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setNodeAttributesCounter);
	LOG(INFO) << "testDistanceConstraints wmNodeCounter.setTransformCounter = " << wmNodeCounter.setTransformCounter;
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send only Atoms with dist < 6 m from me"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(9, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter); // NOTE: this is debatable; here the parent child relation is not send because it would have change the pose too far..
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Transforms with dist > 7 m from me"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(12, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter); // NOTE: this is debatable; here the parent child relation is not send because it would have change the pose too far..
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Transforms with dist > 5 m from me"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(15, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addParentCounter); // NOTE: this is debatable; here the parent child relation is not send because it would have change the pose too far..
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Boxes with dist < 5 m from me"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(18, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addParentCounter); // NOTE: this is debatable; here the parent child relation is not send because it would have change the pose too far..
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send no Meshes with dist > 5.5 m from me"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(21, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addParentCounter); // NOTE: this is debatable; here the parent child relation is not send because it would have change the pose too far..
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.removeParentCounter);

	/* use another UUID */
	vector<Attribute> attributes;
	attributes.clear();
	attributes.push_back(rsg::Attribute("tf:name", "tf_1"));
	vector<Id> results;
	CPPUNIT_ASSERT(wm->scene.getNodes(attributes, results));
	CPPUNIT_ASSERT(results.size() > 0);
	Id nodeId = results[0];
	stringstream c2("");
	c2 << "send no Atoms with dist > 1 m from " << nodeId;
	CPPUNIT_ASSERT(c1.parse(c2.str()));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(21, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addUncertainTransformCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(9, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.setTransformCounter); // THIS is the node used for defining the UUID
	CPPUNIT_ASSERT_EQUAL(9, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.removeParentCounter);

	delete wm;
}

void GraphConstraintTest::testContainmentConstraints() {
	WorldModel* wm = new WorldModel();
	MyObserver wmNodeCounter;
	GraphConstraintUpdateFilter filter(wm);
	wm->scene.attachUpdateObserver(&filter);
	filter.attachUpdateObserver(&wmNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);


	GraphConstraint c1;
	CPPUNIT_ASSERT(c1.parse("send no Atoms contained in me"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter); // in the example attributes are set for another root node (!= me)
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(c1.parse("send only Atoms contained in me"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter); // now this should be skipped
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.deleteNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	delete wm;
}

void GraphConstraintTest::testLODConstraints() {
	WorldModel* wm = new WorldModel();
	MyObserver wmNodeCounter;
	GraphConstraintUpdateFilter filter(wm);
	wm->scene.attachUpdateObserver(&filter);
	filter.attachUpdateObserver(&wmNodeCounter);

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);


	GraphConstraint c1;
	CPPUNIT_ASSERT(c1.parse("send no Atoms with lod > 100"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter); // in the example attributes are set for another root node (!= me)
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);


	CPPUNIT_ASSERT(c1.parse("send only Atoms with lod > 100"));
	filter.constraints.clear();
	filter.constraints.push_back(c1);
	CPPUNIT_ASSERT(runAddAllSceneGraphPrimitives(wm));

	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter); // in the example attributes are set for another root node (!= me)
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.deleteNodeCounter); // never blocked
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	delete wm;
}

bool GraphConstraintTest::runAddAllSceneGraphPrimitives(brics_3d::WorldModel* wm) {

	Id dumyId;
	vector<Attribute> attributes;
	vector<Attribute> dummyAttributes;
	TimeStamp t1 = wm->now();

	CPPUNIT_ASSERT(wm->scene.addNode(wm->getRootNodeId(), dumyId, attributes));
	attributes.clear();
	attributes.push_back(rsg::Attribute("osm:landuse", "forest"));
	CPPUNIT_ASSERT(wm->scene.addNode(wm->getRootNodeId(), dumyId, attributes));
	attributes.clear();
	attributes.push_back(rsg::Attribute("gis:origin", "initial"));
	CPPUNIT_ASSERT(wm->scene.addNode(wm->getRootNodeId(), dumyId, attributes));
	attributes.clear();
	CPPUNIT_ASSERT(wm->scene.addGroup(wm->getRootNodeId(), dumyId, attributes));

	attributes.clear();
	attributes.push_back(rsg::Attribute("tf:name", "tf_1"));
	rsg::Id tf1Id;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3)); 						// Translation coefficients

	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform234(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             2,3,4)); 						// Translation coefficients

	CPPUNIT_ASSERT(wm->scene.addTransformNode(wm->getRootNodeId(), tf1Id, attributes, transform123, t1));

	Id utfId;
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty123(new CovarianceMatrix66(3, 0.001, 0.001, 0.0000, 0.000000, 0.00000));
	CPPUNIT_ASSERT(wm->scene.addUncertainTransformNode(wm->getRootNodeId(), utfId, dummyAttributes, transform234, uncertainty123, wm->now()));

	rsg::Box::BoxPtr box( new rsg::Box(1,2,3));
	rsg::Id boxId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), boxId, dummyAttributes, box, wm->now()));
	CPPUNIT_ASSERT(wm->scene.deleteNode(boxId));

	CPPUNIT_ASSERT(wm->scene.addParent(utfId, tf1Id));
	CPPUNIT_ASSERT(wm->scene.removeParent(utfId, tf1Id));

	Id someRemoteNode = 42+i;
	CPPUNIT_ASSERT(wm->scene.addRemoteRootNode(someRemoteNode, dummyAttributes));

	vector<Attribute> rootAttibutes;
	rootAttibutes.push_back(Attribute("name","someRootNodePolicy"));
	CPPUNIT_ASSERT(wm->scene.setNodeAttributes(someRemoteNode, rootAttibutes, wm->now())); // Note, identical attribute will not be set

	Id connId;
	vector<Attribute> connectionAttributes;
	connectionAttributes.push_back(Attribute("rsg:type","has_geometry"));
	vector<Id> sourceIs;
	sourceIs.push_back(utfId);
	vector<Id> targetIs;
	targetIs.push_back(tf1Id);
	targetIs.push_back(utfId);
	LOG(DEBUG) << "Adding connection { source = {" << utfId << "}, target = {" << tf1Id << ", "<< utfId << "}}";
	CPPUNIT_ASSERT(wm->scene.addConnection(wm->getRootNodeId(), connId, connectionAttributes, sourceIs, targetIs, wm->now(), wm->now()));

	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform456(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             4,5,6));
	TimeStamp t2 = wm->now();//t1 + Duration(0.5, Units::Second);
	CPPUNIT_ASSERT(wm->scene.setTransform(tf1Id, transform456, t2));
	TimeStamp t3 = wm->now();//t2 + Duration(1.0, Units::Second);
	CPPUNIT_ASSERT(wm->scene.setTransform(tf1Id, transform456, t3));

	i++;
	return true;
}

}  // namespace unitTests


/* EOF */
