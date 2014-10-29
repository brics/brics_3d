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

	/* Box for "virtual fence" */
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "box_tf"));
	rsg::Id boxTfId;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform120(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
			0,1,0,
			0,0,1,
			1,2,0)); 						// Translation coefficients
	wm->scene.addTransformNode(sceneObjectsId, boxTfId, attributes, transform120, wm->now());

	doRun = true;
	boost::thread* thread = new boost::thread(boost::bind(&HDF5Test::threadFunction, this, wm));


	int max = 100;
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
			wm->scene.getTransformForNode(*it, wm->getRootNodeId(), creationTime, pose);
			LOG(INFO) << "Pose of box_tf is = " << std::endl << *pose;
		}
	}
	LOG(INFO) << "HDF5Test::threadFunction: stop.";
}

}  // namespace unitTests

/* EOF */
