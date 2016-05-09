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

#define BRICS_JSON_ENABLE  //FIXME
#ifdef BRICS_JSON_ENABLE
#include "JSONTest.h"
#include "SceneGraphNodesTest.h" // for the observer counter
#include "brics_3d/core/Logger.h"
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/TriangleMeshExplicit.h"
#include "brics_3d/worldModel/sceneGraph/DotGraphGenerator.h"
#include <brics_3d/worldModel/sceneGraph/SemanticContextUpdateFilter.h>
#include <brics_3d/worldModel/sceneGraph/UpdatesToSceneGraphListener.h>
#include <brics_3d/worldModel/sceneGraph/JSONSerializer.h>
#include <brics_3d/worldModel/sceneGraph/JSONDeserializer.h>
#include <brics_3d/worldModel/sceneGraph/JSONQueryRunner.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/UuidGenerator.h>
#include <brics_3d/util/JSONTypecaster.h>

#include "boost/thread.hpp"

using namespace brics_3d;

namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( JSONTest );

/*
 * Implementation of data transmission.
 *
 * In a real application this should be replaced by an implementation for
 * a certain communication framework (e.g. ROS, OROCOS, LooCI, sockets, USB, etc.).
 *
 * The JSONSimpleBridge "is" an output port and "has" an input port to directly
 * feed forward data (byte array).
 */
class JSONSimpleBridge : public brics_3d::rsg::IOutputPort {
public:
	JSONSimpleBridge(brics_3d::rsg::IInputPort* inputPort, std::string debugTag = "JSONSimpleBridge") :
		inputPort(inputPort), debugTag(debugTag) {
		LOG(DEBUG) << debugTag << " created.";
	};
	virtual ~JSONSimpleBridge(){};

	int write(const char *dataBuffer, int dataLength, int &transferredBytes) {
		LOG(DEBUG) << debugTag << ": Feeding data forwards.";
		return inputPort->write(dataBuffer, dataLength, transferredBytes); // just feed forward
	};

private:
	brics_3d::rsg::IInputPort* inputPort;
	std::string debugTag;
};


void JSONTest::setUp() {

	doRun = false;
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
}

void JSONTest::tearDown() {
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::WARNING);
}

void JSONTest::testLoopBack() {


	/* Create the world model instances/handles */
	brics_3d::WorldModel* wm = new brics_3d::WorldModel(); 		   // first world model agent
	brics_3d::WorldModel* wmReplica = new brics_3d::WorldModel(); // second world model agent

	/* Attach some (optional) visualization facilities (observers) */
	brics_3d::rsg::VisualizationConfiguration visualizationConfig; // optional configuration
	visualizationConfig.visualizeIds = true;
	visualizationConfig.visualizeAttributes = false;
	visualizationConfig.abbreviateIds = false;

	brics_3d::rsg::DotVisualizer* wmStructureVisualizer = new brics_3d::rsg::DotVisualizer(&wm->scene);
	wmStructureVisualizer->setConfig(visualizationConfig); // we use the same config here
	wmStructureVisualizer->setKeepHistory(false);
	wmStructureVisualizer->setFileName("json_test_graph");
	wm->scene.attachUpdateObserver(wmStructureVisualizer);

	/*
	 * Connect both world model instances via update mechanism as follows:
	 * 1.) some update on first world model
	 * 2.) call attached observer class
	 * 3.) serialize
	 * 4.) send byte steam via JSONSimpleBridge
	 * 5.) deserialize
	 * 6.) according update on second world model
	 */
	brics_3d::rsg::JSONDeserializer* wmUpdatesToJSONdeserializer = new brics_3d::rsg::JSONDeserializer(wmReplica);
//	brics_3d::rsg::JSONDeserializer* wmUpdatesToJSONdeserializer = new brics_3d::rsg::JSONDeserializer(wm/*wmReplica*/);
	JSONSimpleBridge* feedForwardBridge = new JSONSimpleBridge(wmUpdatesToJSONdeserializer);
	brics_3d::rsg::JSONSerializer* wmUpdatesToJSONSerializer = new brics_3d::rsg::JSONSerializer(feedForwardBridge);
	wm->scene.attachUpdateObserver(wmUpdatesToJSONSerializer);
	wmUpdatesToJSONSerializer->setStoreMessageBackupsOnFileSystem(true); /* set to true to store all updates as .h5 files */

	/* Allow roundtrip updates from wmReplica to wm as well */
	brics_3d::rsg::JSONDeserializer* wmUpdatesToJSONdeserializer2 = new brics_3d::rsg::JSONDeserializer(wm);
	JSONSimpleBridge* feedForwardBridge2 = new JSONSimpleBridge(wmUpdatesToJSONdeserializer2, "JSONSimpleBridge-roundtrip");
	brics_3d::rsg::JSONSerializer* wmUpdatesToJSONSerializer2 = new brics_3d::rsg::JSONSerializer(feedForwardBridge2);
	wmReplica->scene.attachUpdateObserver(wmUpdatesToJSONSerializer2);
//	wm/*wmReplica*/->scene.attachUpdateObserver(wmUpdatesToJSONSerializer2);
	wmUpdatesToJSONSerializer2->setStoreMessageBackupsOnFileSystem(false);

	/* Add group nodes */
	std::vector<brics_3d::rsg::Attribute> attributes;
	attributes.clear();
	attributes.push_back(rsg::Attribute("taskType", "scene_objects"));
//	attributes.push_back(rsg::Attribute("some:flag", "executing\": 0"));
	rsg::Id sceneObjectsId;
	return; //FIXME
	wm->scene.addGroup(wm->getRootNodeId(), sceneObjectsId, attributes);

	/* Box TF for "virtual fence" */
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "box_tf"));
	rsg::Id boxTfId;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform120(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
			0,1,0,
			0,0,1,
			1,2,0)); 						// Translation coefficients
	wm->scene.addTransformNode(sceneObjectsId, boxTfId, attributes, transform120, wm->now());

	/* Box for "virtual fence" */
	attributes.clear();
	attributes.push_back(rsg::Attribute("shape", "Box"));
	attributes.push_back(rsg::Attribute("name", "virtual_fence")); // this name serves as a conventions here
	rsg::Box::BoxPtr box( new rsg::Box(1,2,0));
	rsg::Id boxId;
	wm->scene.addGeometricNode(boxTfId, boxId, attributes, box, wm->now());

	doRun = true;
	boost::thread* thread = new boost::thread(boost::bind(&JSONTest::threadFunction, this, wm));


	int max = 10;
	for (int i = 0; i < max; ++i) {
		double x = i/10.0;
		brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr tmpTransform(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
					0,1,0,
					0,0,1,
					x,5.2,5.3)); 						// Translation coefficients
		CPPUNIT_ASSERT(wm->scene.setTransform(boxTfId, tmpTransform, wm->now()));
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr result;
		CPPUNIT_ASSERT(wm->scene.getTransform(boxTfId, wm->now(), result));
		LOG(INFO) << "Freshly inserted transform of box_tf is = " << std::endl << *result;

		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr result2;
		CPPUNIT_ASSERT(wm->scene.getTransform(boxTfId, wmReplica->now(), result2));
		LOG(INFO) << "Freshly inserted and duplicated transform of box_tf is = " << std::endl << *result;

		attributes.clear();
		attributes.push_back(rsg::Attribute("shape", "Box"));
		attributes.push_back(rsg::Attribute("name", "virtual_fence")); // this name serves as a conventions here
		rsg::Box::BoxPtr box( new rsg::Box(1,2,x));
		rsg::Id boxId;
		CPPUNIT_ASSERT(wm->scene.addGeometricNode(boxTfId, boxId, attributes, box, wm->now()));

		attributes.clear();
		attributes.push_back(rsg::Attribute("taskType", "scene_objects"));
		if(i%2 == 1) { // toggling attibutes
			attributes.push_back(rsg::Attribute("some:flag", "executing\": 0"));
		} else {
			attributes.push_back(rsg::Attribute("some:flag", "executing\": 1"));
		}
		CPPUNIT_ASSERT(wm->scene.setNodeAttributes(sceneObjectsId, attributes));
	}


	LOG(INFO) << "JSONTest: Stopping thread.";
	doRun = false;
	thread->join();
	delete thread;
	LOG(INFO) << "JSONTest: Stopping thread done.";

	delete wmUpdatesToJSONSerializer;
	delete wmUpdatesToJSONdeserializer;
	delete feedForwardBridge;
	delete wmReplica;
	delete wm;

//#endif /* BRICS_JSON_ENABLE */

}

void JSONTest::testUpdateObersver() {
	brics_3d::WorldModel* wm = new brics_3d::WorldModel(); 		   // first world model agent
	brics_3d::WorldModel* wmReplica = new brics_3d::WorldModel(); // second world model agent

	vector<Attribute> dummyAttributes;
	vector<Attribute> resultAttributes;
	Id assignedId;

	brics_3d::rsg::JSONDeserializer* wmUpdatesToJSONdeserializer = new brics_3d::rsg::JSONDeserializer(wmReplica);
	JSONSimpleBridge* feedForwardBridge = new JSONSimpleBridge(wmUpdatesToJSONdeserializer);
	brics_3d::rsg::JSONSerializer* wmUpdatesToJSONSerializer = new brics_3d::rsg::JSONSerializer(feedForwardBridge);
	wm->scene.attachUpdateObserver(wmUpdatesToJSONSerializer);
	wmUpdatesToJSONSerializer->setStoreMessageBackupsOnFileSystem(true); /* set to true to store all updates as .h5 files */

	MyObserver wmNodeCounter;
	MyObserver remoteWmNodeCounter;
	wm->scene.attachUpdateObserver(&wmNodeCounter);
	wmReplica->scene.attachUpdateObserver(&remoteWmNodeCounter);

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

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	/* Manually "mount" first wm relative to second one */
	wmReplica->scene.addRemoteRootNode(wm->getRootNodeId(), dummyAttributes);
	wmReplica->scene.addParent(wm->getRootNodeId(), wmReplica->getRootNodeId());

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	/*
	 * Try each API call at least once
	 */
	CPPUNIT_ASSERT(wm->scene.addNode(wm->getRootNodeId(), assignedId, dummyAttributes));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	Id groupId;
	CPPUNIT_ASSERT(wm->scene.addGroup(wm->getRootNodeId(), groupId, dummyAttributes));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	Id tfId;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3));
	CPPUNIT_ASSERT(wm->scene.addTransformNode(wm->getRootNodeId(), tfId, dummyAttributes, transform123, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	Id utfId;
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty123(new CovarianceMatrix66(3, 0.001, 0.001, 0.0000, 0.000000, 0.00000));
	CPPUNIT_ASSERT(wm->scene.addUncertainTransformNode(wm->getRootNodeId(), utfId, dummyAttributes, transform123, uncertainty123, wm->now()));

	rsg::Box::BoxPtr box( new rsg::Box(1,2,3));
	rsg::Id boxId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), boxId, dummyAttributes, box, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	rsg::Sphere::SpherePtr sphere( new rsg::Sphere(4));
	rsg::Id sphereId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), sphereId, dummyAttributes, sphere, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	rsg::Cylinder::CylinderPtr cylinder( new rsg::Cylinder(5,6));
	rsg::Id cylinderId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), cylinderId, dummyAttributes, cylinder, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	/* create some mesh */
	brics_3d::ITriangleMesh::ITriangleMeshPtr newMesh(new brics_3d::TriangleMeshExplicit());
	brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr newMeshContainer(new brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>());
	newMeshContainer->data = newMesh;
	newMeshContainer->data->addTriangle(Point3D(0,0,0), Point3D (0,1,0), Point3D (0,0,3));

	rsg::Id meshId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), meshId, dummyAttributes, newMeshContainer, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);;

	CPPUNIT_ASSERT(wm->scene.addParent(utfId, tfId));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(wm->scene.removeParent(utfId, tfId));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	Id someRemoteNode = 42;
	CPPUNIT_ASSERT(wm->scene.addRemoteRootNode(someRemoteNode, dummyAttributes));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	vector<Attribute> rootAttibutes;
	rootAttibutes.push_back(Attribute("name","someRootNodePolicy"));
	CPPUNIT_ASSERT(wm->scene.setNodeAttributes(wm->getRootNodeId(), rootAttibutes));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	resultAttributes.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultAttributes.size()));
	CPPUNIT_ASSERT(resultAttributes[0].key.compare("name") == 0);
	CPPUNIT_ASSERT(resultAttributes[0].value.compare("someRootNodePolicy") == 0);

	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform456(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             4,5,6));
	CPPUNIT_ASSERT(wm->scene.setTransform(tfId, transform456, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformResult1;
	CPPUNIT_ASSERT(wmReplica->scene.getTransformForNode(tfId, wmReplica->getRootNodeId(), wmReplica->now(), transformResult1));

	//check data
	const double* matrix1 = transformResult1->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4, matrix1[matrixEntry::x], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5, matrix1[matrixEntry::y], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6, matrix1[matrixEntry::z], maxTolerance);

	ITransformUncertainty::ITransformUncertaintyPtr uncertainty456(new CovarianceMatrix66(6, 0.002, 0.002, 0.0000, 0.000000, 0.00000));
	CPPUNIT_ASSERT(wm->scene.setUncertainTransform(utfId, transform456, uncertainty456 , wm->now()));

	CPPUNIT_ASSERT(wm->scene.deleteNode(groupId));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);


	Id connId;
	vector<Attribute> connectionAttributes;
	connectionAttributes.push_back(Attribute("rsg::Type","has_geometry"));
	vector<Id> sourceIs;
	sourceIs.push_back(cylinderId);
	vector<Id> targetIs;
	targetIs.push_back(tfId);
	targetIs.push_back(cylinderId); // TODO utfId ?
	LOG(DEBUG) << "Adding connection { source = {" << cylinderId << "}, target = {" << tfId << ", "<< cylinderId << "}}";
	CPPUNIT_ASSERT(wm->scene.addConnection(wm->getRootNodeId(), connId, connectionAttributes, sourceIs, targetIs, wm->now(), wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	/* Check if connection details have been send correctly */
	vector<Attribute> tmpAttributes;
	tmpAttributes.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getConnectionAttributes(connId, tmpAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(tmpAttributes.size()));
	CPPUNIT_ASSERT(tmpAttributes[0] == Attribute("rsg::Type","has_geometry"));

	vector<Id> resultParentIds;
	resultParentIds.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getConnectionParents(connId, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT_EQUAL(wm->getRootNodeId(), static_cast<Id>(resultParentIds[0]));

	vector<Id> resultIds;
	resultIds.clear();
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("rsg::Type","has_geometry"));
	CPPUNIT_ASSERT(wmReplica->scene.getConnections(tmpAttributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT(resultIds[0] == connId);

	resultIds.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getConnectionSourceIds(connId, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT_EQUAL(cylinderId, static_cast<Id>(resultIds[0]));

	resultIds.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getConnectionTargetIds(connId, resultIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT_EQUAL(tfId, static_cast<Id>(resultIds[0]));
	CPPUNIT_ASSERT_EQUAL(cylinderId, static_cast<Id>(resultIds[1]));

	/* check if insertion of outdated attrutes is handeled */
	vector<Attribute> attributes;
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "myGroup"));
	attributes.push_back(rsg::Attribute("some:flag", "executing\": 0"));
	CPPUNIT_ASSERT(wm->scene.setNodeAttributes(wm->getRootNodeId(), attributes, TimeStamp(1.0, Units::Second)));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "myGroup"));
	attributes.push_back(rsg::Attribute("some:flag", "executing\": 1"));
	CPPUNIT_ASSERT(!wm->scene.setNodeAttributes(wm->getRootNodeId(), attributes, TimeStamp(0.5, Units::Second)));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	//Now lets see if the receiver has the same time stamp information as the sender:
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "myGroup"));
	attributes.push_back(rsg::Attribute("some:flag", "executing\": 2"));
	CPPUNIT_ASSERT(!wmReplica->scene.setNodeAttributes(wm->getRootNodeId(), attributes, TimeStamp(0.5, Units::Second)));

}


void JSONTest::testInputFilter() {
	/*
	 * Test setup:
	 *
	 *    wm --> JSONSerializer --> JSON ---> JSONSimpleBridge -->  JSONDeserializer --> filter --> filterToWM --> wmReplica
	 *
	 */

	brics_3d::WorldModel* wm = new brics_3d::WorldModel(); 		   // first world model agent
	brics_3d::WorldModel* wmReplica = new brics_3d::WorldModel(); // second world model agent

	vector<Attribute> dummyAttributes;
	vector<Attribute> resultAttributes;
	Id assignedId;


	brics_3d::rsg::SemanticContextUpdateFilter filter(&wmReplica->scene); // handle use for queries
	UpdatesToSceneGraphListener filterToWm;
	filterToWm.attachSceneGraph(&wmReplica->scene);
	filter.attachUpdateObserver(&filterToWm); // handle used for updates
	brics_3d::rsg::JSONDeserializer* wmUpdatesToJSONdeserializer = new brics_3d::rsg::JSONDeserializer(wmReplica, &filter);
	filter.setNameSpaceIdentifier("osm");


	JSONSimpleBridge* feedForwardBridge = new JSONSimpleBridge(wmUpdatesToJSONdeserializer);
	brics_3d::rsg::JSONSerializer* wmUpdatesToJSONSerializer = new brics_3d::rsg::JSONSerializer(feedForwardBridge);
	wm->scene.attachUpdateObserver(wmUpdatesToJSONSerializer);
	wmUpdatesToJSONSerializer->setStoreMessageBackupsOnFileSystem(true); /* set to true to store all updates as .h5 files */

	MyObserver wmNodeCounter;
	MyObserver remoteWmNodeCounter;
	wm->scene.attachUpdateObserver(&wmNodeCounter);
	wmReplica->scene.attachUpdateObserver(&remoteWmNodeCounter);

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

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	/* Manually "mount" first wm relative to second one */
	wmReplica->scene.addRemoteRootNode(wm->getRootNodeId(), dummyAttributes);
	wmReplica->scene.addParent(wm->getRootNodeId(), wmReplica->getRootNodeId());

	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addNodeCounter); //precondition
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	/*
	 * Try each API call at least once
	 */
	CPPUNIT_ASSERT(wm->scene.addNode(wm->getRootNodeId(), assignedId, dummyAttributes));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	Id groupId;
	CPPUNIT_ASSERT(wm->scene.addGroup(wm->getRootNodeId(), groupId, dummyAttributes));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	Id tfId;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             1,2,3));
	CPPUNIT_ASSERT(wm->scene.addTransformNode(wm->getRootNodeId(), tfId, dummyAttributes, transform123, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	Id utfId;
	ITransformUncertainty::ITransformUncertaintyPtr uncertainty123(new CovarianceMatrix66(3, 0.001, 0.001, 0.0000, 0.000000, 0.00000));
	CPPUNIT_ASSERT(wm->scene.addUncertainTransformNode(wm->getRootNodeId(), utfId, dummyAttributes, transform123, uncertainty123, wm->now()));

	rsg::Box::BoxPtr box( new rsg::Box(1,2,3));
	rsg::Id boxId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), boxId, dummyAttributes, box, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	rsg::Sphere::SpherePtr sphere( new rsg::Sphere(4));
	rsg::Id sphereId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), sphereId, dummyAttributes, sphere, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	rsg::Cylinder::CylinderPtr cylinder( new rsg::Cylinder(5,6));
	rsg::Id cylinderId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), cylinderId, dummyAttributes, cylinder, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	/* create some mesh */
	brics_3d::ITriangleMesh::ITriangleMeshPtr newMesh(new brics_3d::TriangleMeshExplicit());
	brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr newMeshContainer(new brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>());
	newMeshContainer->data = newMesh;
	newMeshContainer->data->addTriangle(Point3D(0,0,0), Point3D (0,1,0), Point3D (0,0,3));

	rsg::Id meshId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), meshId, dummyAttributes, newMeshContainer, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);;

	CPPUNIT_ASSERT(wm->scene.addParent(meshId, tfId));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(wm->scene.removeParent(meshId, tfId));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	Id someRemoteNode = 42;
	CPPUNIT_ASSERT(wm->scene.addRemoteRootNode(someRemoteNode, dummyAttributes));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	vector<Attribute> rootAttibutes;
	rootAttibutes.push_back(Attribute("name","someRootNodePolicy"));
	CPPUNIT_ASSERT(wm->scene.setNodeAttributes(wm->getRootNodeId(), rootAttibutes));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	resultAttributes.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultAttributes.size()));
	CPPUNIT_ASSERT(resultAttributes[0].key.compare("name") == 0);
	CPPUNIT_ASSERT(resultAttributes[0].value.compare("someRootNodePolicy") == 0);

	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform456(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             4,5,6));
	CPPUNIT_ASSERT(wm->scene.setTransform(tfId, transform456, wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformResult1;
	CPPUNIT_ASSERT(wmReplica->scene.getTransformForNode(tfId, wmReplica->getRootNodeId(), wmReplica->now(), transformResult1));

	//check data
	const double* matrix1 = transformResult1->getRawData();
	CPPUNIT_ASSERT_DOUBLES_EQUAL(4, matrix1[matrixEntry::x], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(5, matrix1[matrixEntry::y], maxTolerance);
	CPPUNIT_ASSERT_DOUBLES_EQUAL(6, matrix1[matrixEntry::z], maxTolerance);

	ITransformUncertainty::ITransformUncertaintyPtr uncertainty456(new CovarianceMatrix66(6, 0.002, 0.002, 0.0000, 0.000000, 0.00000));
	CPPUNIT_ASSERT(wm->scene.setUncertainTransform(utfId, transform456, uncertainty456 , wm->now()));

	CPPUNIT_ASSERT(wm->scene.deleteNode(groupId));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);


	Id connId;
	vector<Attribute> connectionAttributes;
	connectionAttributes.push_back(Attribute("rsg::Type","has_geometry"));
	vector<Id> sourceIs;
	sourceIs.push_back(cylinderId);
	vector<Id> targetIs;
	targetIs.push_back(tfId);
	targetIs.push_back(cylinderId); // TODO utfId ?
	LOG(DEBUG) << "Adding connection { source = {" << cylinderId << "}, target = {" << tfId << ", "<< cylinderId << "}}";
	CPPUNIT_ASSERT(wm->scene.addConnection(wm->getRootNodeId(), connId, connectionAttributes, sourceIs, targetIs, wm->now(), wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	/* Check if connection details have been send correctly */
	vector<Attribute> tmpAttributes;
	tmpAttributes.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getConnectionAttributes(connId, tmpAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(tmpAttributes.size()));
	CPPUNIT_ASSERT(tmpAttributes[0] == Attribute("rsg::Type","has_geometry"));

	vector<Id> resultParentIds;
	resultParentIds.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getConnectionParents(connId, resultParentIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultParentIds.size()));
	CPPUNIT_ASSERT_EQUAL(wm->getRootNodeId(), static_cast<Id>(resultParentIds[0]));

	vector<Id> resultIds;
	resultIds.clear();
	tmpAttributes.clear();
	tmpAttributes.push_back(Attribute("rsg::Type","has_geometry"));
	CPPUNIT_ASSERT(wmReplica->scene.getConnections(tmpAttributes, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT(resultIds[0] == connId);

	resultIds.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getConnectionSourceIds(connId, resultIds));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT_EQUAL(cylinderId, static_cast<Id>(resultIds[0]));

	resultIds.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getConnectionTargetIds(connId, resultIds));
	CPPUNIT_ASSERT_EQUAL(2u, static_cast<unsigned int>(resultIds.size()));
	CPPUNIT_ASSERT_EQUAL(tfId, static_cast<Id>(resultIds[0]));
	CPPUNIT_ASSERT_EQUAL(cylinderId, static_cast<Id>(resultIds[1]));

	/* check if insertion of outdated attrutes is handeled */
	vector<Attribute> attributes;
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "myGroup"));
	attributes.push_back(rsg::Attribute("some:flag", "executing\": 0"));
	CPPUNIT_ASSERT(wm->scene.setNodeAttributes(wm->getRootNodeId(), attributes, TimeStamp(1.0, Units::Second)));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "myGroup"));
	attributes.push_back(rsg::Attribute("some:flag", "executing\": 1"));
	CPPUNIT_ASSERT(!wm->scene.setNodeAttributes(wm->getRootNodeId(), attributes, TimeStamp(0.5, Units::Second)));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);


	/*
	 * Now we repeat the same but put an attribute that has to be filtered out
	 */
	dummyAttributes.clear();
	dummyAttributes.push_back(Attribute("osm:name", "This should be filtered."));

	CPPUNIT_ASSERT(wm->scene.addNode(wm->getRootNodeId(), assignedId, dummyAttributes));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

//	Id groupId;
	CPPUNIT_ASSERT(wm->scene.addGroup(wm->getRootNodeId(), groupId, dummyAttributes));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

//	Id tfId;
//	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform123(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
//	                                                             0,1,0,
//	                                                             0,0,1,
//	                                                             1,2,3));
	CPPUNIT_ASSERT(wm->scene.addTransformNode(wm->getRootNodeId(), tfId, dummyAttributes, transform123, wm->now()));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

//	Id utfId;
//	ITransformUncertainty::ITransformUncertaintyPtr uncertainty123(new CovarianceMatrix66(3, 0.001, 0.001, 0.0000, 0.000000, 0.00000));
	CPPUNIT_ASSERT(wm->scene.addUncertainTransformNode(wm->getRootNodeId(), utfId, dummyAttributes, transform123, uncertainty123, wm->now()));

//	rsg::Box::BoxPtr box( new rsg::Box(1,2,3));
//	rsg::Id boxId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), boxId, dummyAttributes, box, wm->now()));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

//	rsg::Sphere::SpherePtr sphere( new rsg::Sphere(4));
//	rsg::Id sphereId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), sphereId, dummyAttributes, sphere, wm->now()));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(6, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

//	rsg::Cylinder::CylinderPtr cylinder( new rsg::Cylinder(5,6));
//	rsg::Id cylinderId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), cylinderId, dummyAttributes, cylinder, wm->now()));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(7, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	/* create some mesh */
//	brics_3d::ITriangleMesh::ITriangleMeshPtr newMesh(new brics_3d::TriangleMeshExplicit());
//	brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr newMeshContainer(new brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>());
//	newMeshContainer->data = newMesh;
//	newMeshContainer->data->addTriangle(Point3D(0,0,0), Point3D (0,1,0), Point3D (0,0,3));

//	rsg::Id meshId;
	CPPUNIT_ASSERT(wm->scene.addGeometricNode(wm->getRootNodeId(), meshId, dummyAttributes, newMeshContainer, wm->now()));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(wm->scene.addParent(meshId, tfId));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(wm->scene.removeParent(meshId, tfId));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);
	someRemoteNode = 43;
	CPPUNIT_ASSERT(wm->scene.addRemoteRootNode(someRemoteNode, dummyAttributes));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

//	vector<Attribute> rootAttibutes;
	rootAttibutes.clear();
	rootAttibutes.push_back(Attribute("name","someRootNodePolicy"));
	CPPUNIT_ASSERT(wm->scene.setNodeAttributes(wm->getRootNodeId(), rootAttibutes, TimeStamp(1.5, Units::Second)));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	resultAttributes.clear();
	CPPUNIT_ASSERT(wmReplica->scene.getNodeAttributes(wm->getRootNodeId(), resultAttributes));
	CPPUNIT_ASSERT_EQUAL(1u, static_cast<unsigned int>(resultAttributes.size()));
	CPPUNIT_ASSERT(resultAttributes[0].key.compare("name") == 0);
	CPPUNIT_ASSERT(resultAttributes[0].value.compare("someRootNodePolicy") == 0);

//	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform456(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
//	                                                             0,1,0,
//	                                                             0,0,1,
//	                                                             4,5,6));
	CPPUNIT_ASSERT(wm->scene.setTransform(tfId, transform456, wm->now()));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT(wm->scene.setUncertainTransform(utfId, transform456, uncertainty456 , wm->now()));

	CPPUNIT_ASSERT(wm->scene.deleteNode(groupId));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);


//	Id connId;
//	vector<Attribute> connectionAttributes;
	connectionAttributes.clear();
	connectionAttributes.push_back(Attribute("osm::Type","has_geometry"));
//	vector<Id> sourceIs;
	sourceIs.clear();
	sourceIs.push_back(cylinderId);
//	vector<Id> targetIs;
	targetIs.clear();
	targetIs.push_back(tfId);
	targetIs.push_back(cylinderId); // TODO utfId ?
	LOG(DEBUG) << "Adding connection { source = {" << cylinderId << "}, target = {" << tfId << ", "<< cylinderId << "}}";
	CPPUNIT_ASSERT(wm->scene.addConnection(wm->getRootNodeId(), connId, connectionAttributes, sourceIs, targetIs, wm->now(), wm->now()));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

	vector<Attribute> someAttributes;
	someAttributes.clear();
	someAttributes.push_back(Attribute("comment","This is rejected because the node has _previousley stored the osm prefix."));
	CPPUNIT_ASSERT(wm->scene.setNodeAttributes(tfId, someAttributes, TimeStamp(2, Units::Second)));

	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(8, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(5, wmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(2, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(4, remoteWmNodeCounter.setNodeAttributesCounter); // NOTE: callObserversEvenIfErrorsOccurred is true as default, which is the case here
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

}

void JSONTest::threadFunction(brics_3d::WorldModel* wm) {
	LOG(INFO) << "JSONTest::threadFunction: start.";

	vector<rsg::Attribute> queryAttributes;
	vector<rsg::Id> resultIds;

	/* Get data of obstacles (spheres) */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(rsg::Attribute("name", "box_tf"));
	wm->scene.getNodes(queryAttributes, resultIds);

	for(vector<rsg::Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
		rsg::TimeStamp creationTime = wm->now();
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr pose;
		wm->scene.getTransformForNode(*it, wm->getRootNodeId(), creationTime, pose);
		LOG(INFO) << "Pose of box_tf is = " << std::endl << *pose;
	}



	while(doRun) {
		LOG(INFO) << "JSONTest::threadFunction: run.";

		/* Get data of obstacles (spheres) */
		wm->scene.getNodes(queryAttributes, resultIds);

		for(vector<rsg::Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
			rsg::TimeStamp creationTime = wm->now();
			IHomogeneousMatrix44::IHomogeneousMatrix44Ptr pose;
			CPPUNIT_ASSERT(wm->scene.getTransformForNode(*it, wm->getRootNodeId(), creationTime, pose));
			LOG(INFO) << "Pose of box_tf is = " << std::endl << *pose;
		}
	}
	LOG(INFO) << "JSONTest::threadFunction: stop.";
}

void JSONTest::testQuerys() {
	Id rootId;
	rootId.fromString("00000000-0000-0000-0000-000000000042");
	rsg::IIdGenerator* idGenerator = new brics_3d::rsg::UuidGenerator(rootId);
	brics_3d::WorldModel* wm = new brics_3d::WorldModel(idGenerator);
	brics_3d::rsg::JSONQueryRunner queryRunner(wm);

	std::string queryAsJson = "";
	std::string resultAsJson = "";

	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Top level model type @worldmodeltype does not exist.\"}}") == 0);

	queryAsJson = "xdkvmsdpj0ßr98w3ß9+ vrsßvvr8ameg+ß9vw4 ";
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Top level model type @worldmodeltype does not exist.\"}}") == 0);


	std::stringstream queryAsJson2;
	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_NODE_ATTRIBUTES\","
	    << "\"id\": \"d0483c43-4a36-4197-be49-de829cdd66c9\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson)); // correctly parsed, but the node does not exist.
	CPPUNIT_ASSERT(resultAsJson.compare("{\"@worldmodeltype\": \"RSGQueryResult\",\"query\": \"GET_NODE_ATTRIBUTES\",\"querySuccess\": false}") == 0); // "{}" means parser error;

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_NODE_ATTRIBUTES\","
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson)); // missing id
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Wrong or missing id.\"}}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
	    << "\"id\": \"d0483c43-4a36-4197-be49-de829cdd66c9\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson)); // missing query
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Mandatory query field not set in RSGQuery\"}}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"INVALID query\","
	    << "\"id\": \"d0483c43-4a36-4197-be49-de829cdd66c9\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson)); // missing query
	LOG(DEBUG) << "resultAsJson invalid query " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Mandatory query field has unknown value in RSGQuery\"}}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQueryINVALID\","
	    << "\"id\": \"d0483c43-4a36-4197-be49-de829cdd66c9\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson)); // missing query
	LOG(DEBUG) << "resultAsJson invalid type " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Mandatory @worldmodeltype field not set in RSGQuery\"}}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"query\": \"GET_NODE_ATTRIBUTES\","
	    << "\"id\": \"d0483c43-4a36-4197-be49-de829cdd66c9\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson)); // missing @worldmodeltype
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Top level model type @worldmodeltype does not exist.\"}}") == 0);


	queryAsJson = "xdkvmsdpj0ßr98w3ß9+ vrsßvvr8ameg+ß9vw4 ";
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Top level model type @worldmodeltype does not exist.\"}}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_NODE_ATTRIBUTES\","
	    << "\"id\": \"INVALID-ID\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson)); // correctly parsed, but the node does not exist.
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Wrong or missing id.\"}}") == 0);

	/*
	 * Now go for some other query types
	 */

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_NODE_PARENTS\","
	    << "\"id\": \"INVALID-ID\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Wrong or missing id.\"}}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_GROUP_CHILDREN\","
	    << "\"id\": \"INVALID-ID\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Wrong or missing id.\"}}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_ROOT_NODE\","
	    << "\"id\": \"INVALID-ID\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(queryRunner.query(queryAsJson, resultAsJson)); // id is not required, thus an invalid one is ignored
	LOG(DEBUG) << "resultAsJson root id" << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"@worldmodeltype\": \"RSGQueryResult\",\"query\": \"GET_ROOT_NODE\",\"querySuccess\": true,\"rootId\": \"00000000-0000-0000-0000-000000000042\"}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_REMOTE_ROOT_NODES\","
	    << "\"id\": \"INVALID-ID\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(queryRunner.query(queryAsJson, resultAsJson));  // id is not required, thus an invalid one is ignored
	LOG(DEBUG) << "resultAsJson remote roots" << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"@worldmodeltype\": \"RSGQueryResult\",\"ids\": [],\"query\": \"GET_REMOTE_ROOT_NODES\",\"querySuccess\": true}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_TRANSFORM\","
	    << "\"id\": \"INVALID-ID\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Wrong or missing id.\"}}") == 0);


	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_GEOMETRY\","
	    << "\"id\": \"INVALID-ID\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Wrong or missing id.\"}}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_CONNECTION_SOURCE_IDS\","
	    << "\"id\": \"INVALID-ID\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Wrong or missing id.\"}}") == 0);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_CONNECTION_TARGET_IDS\","
	    << "\"id\": \"INVALID-ID\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson)); // correctly parsed, but the node does not exist.
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"error\": {\"message\": \"Syntax error: Wrong or missing id.\"}}") == 0);

	/*
	 * Well formatted but false quieries
	 */

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_NODE_PARENTS\","
	    << "\"id\": \"d0483c43-4a36-4197-be49-de829cdd66c9\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"@worldmodeltype\": \"RSGQueryResult\",\"ids\": [],\"query\": \"GET_NODE_PARENTS\",\"querySuccess\": false}") == 0);

	/*
	 * Now go for some working queries
	 */

	Id id;
	CPPUNIT_ASSERT(id.fromString("d0483c43-4a36-4197-be49-de829cdd66c9"));
	vector<Attribute> attributes;
	wm->scene.addNode(wm->getRootNodeId(),id, attributes, true);

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_NODE_ATTRIBUTES\","
	    << "\"id\": \"d0483c43-4a36-4197-be49-de829cdd66c9\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(queryRunner.query(queryAsJson, resultAsJson)); // This should work
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"@worldmodeltype\": \"RSGQueryResult\",\"attributes\": [],\"query\": \"GET_NODE_ATTRIBUTES\",\"querySuccess\": true}") == 0); // "{}" means parser error;

	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_NODE_PARENTS\","
	    << "\"id\": \"d0483c43-4a36-4197-be49-de829cdd66c9\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"@worldmodeltype\": \"RSGQueryResult\",\"ids\": [\"00000000-0000-0000-0000-000000000042\"],\"query\": \"GET_NODE_PARENTS\",\"querySuccess\": true}") == 0);

	/*
	 * Query with queryId
	 */
	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGQuery\","
		<< "\"query\": \"GET_NODE_PARENTS\","
		<< "\"queryId\": \"some-query-id-12345\","
	    << "\"id\": \"d0483c43-4a36-4197-be49-de829cdd66c9\""
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"@worldmodeltype\": \"RSGQueryResult\",\"ids\": [\"00000000-0000-0000-0000-000000000042\"],\"query\": \"GET_NODE_PARENTS\",\"queryId\": \"some-query-id-12345\",\"querySuccess\": true}") == 0);


	/*
	 * Broken query
	 */
	queryAsJson2.str("");
	queryAsJson2
	<<"{{{{{{{{{{{{{{{{{{{{{{{{{{{{";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{}") == 0);


	delete wm;
}

void JSONTest::testFunctionBlockQuerys() {
	Id rootId;
	rootId.fromString("00000000-0000-0000-0000-000000000042");
	rsg::IIdGenerator* idGenerator = new brics_3d::rsg::UuidGenerator(rootId);
	brics_3d::WorldModel* wm = new brics_3d::WorldModel(idGenerator);
	brics_3d::rsg::JSONQueryRunner queryRunner(wm);

	std::string queryAsJson = "";
	std::string resultAsJson = "";

	std::stringstream queryAsJson2;
	queryAsJson2.str("");
	queryAsJson2
	<<"{"
		<< "\"@worldmodeltype\": \"RSGFunctionBlock\","
		<< "\"metamodel\":       \"rsg-functionBlock-schema.json\","
		<< "\"name\":            \"roifilter\","
		<< "\"operation\":       \"EXECUTE\","
		<< "\"input\": ["
		<< "	\"943ba6f4-5c70-46ec-83af-0d5434953e5f\","
		<< "	\"631ba6f4-5c70-46ec-83af-0d5434953e5f\""
		<<   "]"
	<<"}";
	queryAsJson = queryAsJson2.str();
	CPPUNIT_ASSERT(!queryRunner.query(queryAsJson, resultAsJson));
	LOG(DEBUG) << "resultAsJson " << resultAsJson;
	CPPUNIT_ASSERT(resultAsJson.compare("{\"@worldmodeltype\": \"RSGFunctionBlockResult\",\"metamodel\": \"rsg-functionBlock-schema.json\",\"operation\": \"EXECUTE\",\"operationSuccess\": false,\"output\": []}") == 0); // "{}" means parser error;

	delete wm;
}

}  // namespace unitTests

#endif /* BRICS_JSON_ENABLE */

/* EOF */
