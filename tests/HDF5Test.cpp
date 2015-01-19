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

#include "HDF5Test.h"
#include "SceneGraphNodesTest.h" // for the observer counter
#include "brics_3d/core/Logger.h"
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/worldModel/sceneGraph/DotGraphGenerator.h"

#include <brics_3d/worldModel/sceneGraph/HDF5UpdateSerializer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateDeserializer.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>
#include <brics_3d/util/HDF5Typecaster.h>
#include <hdf5.h>

#include "boost/thread.hpp"

using namespace brics_3d;

namespace unitTests {

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( HDF5Test );

/*
 * Implementation of data transmission.
 *
 * In a real application this should be replaced by an implementation for
 * a certain communication framework (e.g. ROS, OROCOS, LooCI, sockets, USB, etc.).
 *
 * The HSDF5SimleBridge "is" an output port and "has" an input port to directly
 * feed forward data (byte array).
 */
class HSDF5SimleBridge : public brics_3d::rsg::IOutputPort {
public:
	HSDF5SimleBridge(brics_3d::rsg::IInputPort* inputPort, std::string debugTag = "HSDF5SimleBridge") :
		inputPort(inputPort), debugTag(debugTag) {
		LOG(DEBUG) << debugTag << " created.";
	};
	virtual ~HSDF5SimleBridge(){};

	int write(const char *dataBuffer, int dataLength, int &transferredBytes) {
		LOG(DEBUG) << debugTag << ": Feeding data forwards.";
//		hexdump(dataBuffer, dataLength);
		return inputPort->write(dataBuffer, dataLength, transferredBytes); // just feed forward
	};

private:
	std::string debugTag;
	brics_3d::rsg::IInputPort* inputPort;
};


void HDF5Test::setUp() {

	doRun = false;
//	wm = new brics_3d::WorldModel();
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);
}

void HDF5Test::tearDown() {
//	delete wm;
//	wm = 0;
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::WARNING);
}

void HDF5Test::testLoopBack() {

	//#ifdef BRICS_HDF5_ENABLE

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
	wm->scene.attachUpdateObserver(wmStructureVisualizer);

	/*
	 * Connect both world model instances via update mechanism as follows:
	 * 1.) some update on first world model
	 * 2.) call attached observer class
	 * 3.) serialize
	 * 4.) send byte steam via HSDF5SimleBridge
	 * 5.) deserialize
	 * 6.) according update on second world model
	 */
	brics_3d::rsg::HDF5UpdateDeserializer* wmUpdatesToHdf5deserializer = new brics_3d::rsg::HDF5UpdateDeserializer(wmReplica);
//	brics_3d::rsg::HDF5UpdateDeserializer* wmUpdatesToHdf5deserializer = new brics_3d::rsg::HDF5UpdateDeserializer(wm/*wmReplica*/);
	HSDF5SimleBridge* feedForwardBridge = new HSDF5SimleBridge(wmUpdatesToHdf5deserializer);
	brics_3d::rsg::HDF5UpdateSerializer* wmUpdatesToHdf5Serializer = new brics_3d::rsg::HDF5UpdateSerializer(feedForwardBridge);
	wm->scene.attachUpdateObserver(wmUpdatesToHdf5Serializer);
	wmUpdatesToHdf5Serializer->setStoreMessageBackupsOnFileSystem(true); /* set to true to store all updates as .h5 files */

	/* Allow roundtrip updates from wmReplica to wm as well */
	brics_3d::rsg::HDF5UpdateDeserializer* wmUpdatesToHdf5deserializer2 = new brics_3d::rsg::HDF5UpdateDeserializer(wm);
	HSDF5SimleBridge* feedForwardBridge2 = new HSDF5SimleBridge(wmUpdatesToHdf5deserializer2, "HSDF5SimleBridge-roundtrip");
	brics_3d::rsg::HDF5UpdateSerializer* wmUpdatesToHdf5Serializer2 = new brics_3d::rsg::HDF5UpdateSerializer(feedForwardBridge2);
	wmReplica->scene.attachUpdateObserver(wmUpdatesToHdf5Serializer2);
//	wm/*wmReplica*/->scene.attachUpdateObserver(wmUpdatesToHdf5Serializer2);
	wmUpdatesToHdf5Serializer2->setStoreMessageBackupsOnFileSystem(false);

	/* Add group nodes */
	std::vector<brics_3d::rsg::Attribute> attributes;
	attributes.clear();
	attributes.push_back(rsg::Attribute("taskType", "scene_objecs"));
	rsg::Id sceneObjectsId;
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
	boost::thread* thread = new boost::thread(boost::bind(&HDF5Test::threadFunction, this, wm));


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
	}


	LOG(INFO) << "HDF5Test: Stopping thread.";
	doRun = false;
	thread->join();
	delete thread;
	LOG(INFO) << "HDF5Test: Stopping thread done.";

	delete wmUpdatesToHdf5Serializer;
	delete wmUpdatesToHdf5deserializer;
	delete feedForwardBridge;
	delete wmReplica;
	delete wm;

//#endif /* BRICS_HDF5_ENABLE */

}

void HDF5Test::testUpdateObersver() {
	brics_3d::WorldModel* wm = new brics_3d::WorldModel(); 		   // first world model agent
	brics_3d::WorldModel* wmReplica = new brics_3d::WorldModel(); // second world model agent

	vector<Attribute> dummyAttributes;
	vector<Attribute> resultAttributes;
	Id assignedId;

	brics_3d::rsg::HDF5UpdateDeserializer* wmUpdatesToHdf5deserializer = new brics_3d::rsg::HDF5UpdateDeserializer(wmReplica);
	HSDF5SimleBridge* feedForwardBridge = new HSDF5SimleBridge(wmUpdatesToHdf5deserializer);
	brics_3d::rsg::HDF5UpdateSerializer* wmUpdatesToHdf5Serializer = new brics_3d::rsg::HDF5UpdateSerializer(feedForwardBridge);
	wm->scene.attachUpdateObserver(wmUpdatesToHdf5Serializer);
	wmUpdatesToHdf5Serializer->setStoreMessageBackupsOnFileSystem(true); /* set to true to store all updates as .h5 files */

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

	CPPUNIT_ASSERT(wm->scene.addParent(utfId, tfId));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addGeometricNodeCounter);
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
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addGeometricNodeCounter);
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
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addGeometricNodeCounter);
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
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addGeometricNodeCounter);
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
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(0, wmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.removeParentCounter);

	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addGeometricNodeCounter);
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
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
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
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addGeometricNodeCounter);
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
	CPPUNIT_ASSERT(wm->scene.addConnection(wm->getRootNodeId(), connId, connectionAttributes, sourceIs, targetIs, wm->now(), wm->now()));

	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addGroupCounter);
	CPPUNIT_ASSERT_EQUAL(1, wmNodeCounter.addTransformCounter);
	CPPUNIT_ASSERT_EQUAL(3, wmNodeCounter.addGeometricNodeCounter);
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
	CPPUNIT_ASSERT_EQUAL(3, remoteWmNodeCounter.addGeometricNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addRemoteRootNodeCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.addConnectionCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setNodeAttributesCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.setTransformCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.deleteNodeCounter);
	CPPUNIT_ASSERT_EQUAL(2, remoteWmNodeCounter.addParentCounter);
	CPPUNIT_ASSERT_EQUAL(1, remoteWmNodeCounter.removeParentCounter);

}

void HDF5Test::threadFunction(brics_3d::WorldModel* wm) {
	LOG(INFO) << "HDF5Test::threadFunction: start.";

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
		LOG(INFO) << "HDF5Test::threadFunction: run.";

		/* Get data of obstacles (spheres) */
		wm->scene.getNodes(queryAttributes, resultIds);

		for(vector<rsg::Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
			rsg::TimeStamp creationTime = wm->now();
			IHomogeneousMatrix44::IHomogeneousMatrix44Ptr pose;
			CPPUNIT_ASSERT(wm->scene.getTransformForNode(*it, wm->getRootNodeId(), creationTime, pose));
			LOG(INFO) << "Pose of box_tf is = " << std::endl << *pose;
		}
	}
	LOG(INFO) << "HDF5Test::threadFunction: stop.";
}

}  // namespace unitTests

/* EOF */
