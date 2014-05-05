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

/*
 * This example illustrates a distributed world model with two world model
 * instances involved (aka delegates).
 *
 * The second instance is an exact "replica" of the first one.
 * In this case all updates on the world model will be forwarded to the replica
 * as well. A single update involves "serialization" via HDF5, transmission of
 * data (byte stream) and deserialization on the reciever's side.
 *
 *  +----------+                +-------------+
 *  |    wm    |  ---------->   |  wmReplica  |
 *  +----------+    updates     +-------------+
 *
 * As an example the world model instance could be deployed on a tablet PC to
 * control a robot. The world model replica is then used by a particular robot
 * to influence its internal base motion controller.
 *
 * We assume the motion controller needs spheres as obstacles and a bounding box
 * a virtual fence.
 *
 */

/* BRICS_3D includes */
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateSerializer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateDeserializer.h>

#include <brics_3d/util/HDF5Typecaster.h>
#include <hdf5.h>
using namespace brics_3d;
using brics_3d::Logger;

// Just a helper tool to print an update message
void hexdump(const char *ptr, int buflen) {
	unsigned char *buf = (unsigned char*)ptr;
	int i, j;
	for (i=0; i<buflen; i+=16) {
		printf("%06x: ", i);
		for (j=0; j<16; j++)
			if (i+j < buflen)
				printf("%02x ", buf[i+j]);
			else
				printf("   ");
		printf(" ");
		for (j=0; j<16; j++)
			if (i+j < buflen)
				printf("%c", isprint(buf[i+j]) ? buf[i+j] : '.');
		printf("\n");
	}
}

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
	HSDF5SimleBridge(brics_3d::rsg::IInputPort* inputPort) : inputPort(inputPort){};
	virtual ~HSDF5SimleBridge(){};

	int write(const char *dataBuffer, int dataLength, int &transferredBytes) {
		LOG(DEBUG) << "HSDF5SimleBridge: Feeding data forwards.";
//		hexdump(dataBuffer, dataLength);
		return inputPort->write(dataBuffer, dataLength, transferredBytes); // just feed forward
	};

private:
	brics_3d::rsg::IInputPort* inputPort;
};

int main(int argc, char **argv) {

	/* Configure the logger - default level won't tell us much */
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	/* Create the world model instances/handles */
	brics_3d::WorldModel* wm = new brics_3d::WorldModel();        // first delegate
	brics_3d::WorldModel* wmReplica = new brics_3d::WorldModel(); // second delegate

	/* Attach some (optional) visualization facilities (observers) */
	brics_3d::rsg::OSGVisualizer* wm3DVisualizer = new brics_3d::rsg::OSGVisualizer();
	brics_3d::rsg::DotVisualizer* wmStructureVisualizer = new brics_3d::rsg::DotVisualizer(&wm->scene);
	brics_3d::rsg::DotVisualizer* wmReplicaStructureVisualizer = new brics_3d::rsg::DotVisualizer(&wmReplica->scene);
	brics_3d::rsg::VisualizationConfiguration osgConfiguration; // optional configuration
	osgConfiguration.visualizeIds = true;
	osgConfiguration.visualizeAttributes = false;
	osgConfiguration.abbreviateIds = true;
	wm3DVisualizer->setConfig(osgConfiguration);
	wmStructureVisualizer->setConfig(osgConfiguration); // we use the same config here
	wmStructureVisualizer->setKeepHistory(true);
	wmReplicaStructureVisualizer->setConfig(osgConfiguration);
	wmReplicaStructureVisualizer->setFileName("current_replica_graph");
	wmReplicaStructureVisualizer->setKeepHistory(true);
	wm->scene.attachUpdateObserver(wm3DVisualizer); //enable visualization
	wm->scene.attachUpdateObserver(wmStructureVisualizer);
	wmReplica->scene.attachUpdateObserver(wmReplicaStructureVisualizer);
	usleep(500*1000); // the OSG visualization window seems to need some setup time

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
	HSDF5SimleBridge* feedForwardBridge = new HSDF5SimleBridge(wmUpdatesToHdf5deserializer);
	brics_3d::rsg::HDF5UpdateSerializer* wmUpdatesToHdf5Serializer = new brics_3d::rsg::HDF5UpdateSerializer(feedForwardBridge);
	wm->scene.attachUpdateObserver(wmUpdatesToHdf5Serializer);


	/* ================== Setup of world model content ===================== */

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

	attributes.clear();
	attributes.push_back(rsg::Attribute("shape", "Box"));
	attributes.push_back(rsg::Attribute("name", "virtual_fence")); // this name serves as a conventions here
	rsg::Box::BoxPtr box( new rsg::Box(1,2,0));
	rsg::Id boxId;
	wm->scene.addGeometricNode(boxTfId, boxId, attributes, box, wm->now());

	/* A sphere used as obstacle  */
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "shpere_1_tf"));
	rsg::Id sphere1TfId;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformShpere1(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0.5,1,0)); 						// Translation coefficients
	wm->scene.addTransformNode(sceneObjectsId, sphere1TfId, attributes, transformShpere1, wm->now());

	attributes.clear();
	attributes.push_back(rsg::Attribute("shape", "Sphere"));
	attributes.push_back(rsg::Attribute("name", "sphere_1"));
	rsg::Sphere::SpherePtr sphere1( new rsg::Sphere(20, Units::CentiMeter));
	rsg::Id sphere1Id;
	wm->scene.addGeometricNode(sphere1TfId, sphere1Id, attributes, sphere1, wm->now());

	/* A another sphere used as obstacle  */
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "shpere_2_tf"));
	rsg::Id sphere2TfId;
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformShpere2(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0.8,1.5,0)); 						// Translation coefficients
	wm->scene.addTransformNode(sceneObjectsId, sphere2TfId, attributes, transformShpere2, wm->now());

	attributes.clear();
	attributes.push_back(rsg::Attribute("shape", "Sphere"));
	attributes.push_back(rsg::Attribute("name", "sphere_2"));
	rsg::Sphere::SpherePtr sphere2( new rsg::Sphere(10, Units::CentiMeter));
	rsg::Id sphere2Id;
	wm->scene.addGeometricNode(sphere2TfId, sphere2Id, attributes, sphere2, wm->now());

	LOG(INFO) << "Done with setup of world model.";

	/* ============ Sample queries by the motion controller ================ */

	vector<rsg::Attribute> queryAttributes;
	vector<rsg::Id> resultIds;

	/* Get data and pose of the box */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(rsg::Attribute("name", "virtual_fence"));
	wmReplica->scene.getNodes(queryAttributes, resultIds);

	for(vector<rsg::Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
		rsg::TimeStamp creationTime;
		rsg::Shape::ShapePtr shape;
		wmReplica->scene.getGeometry(*it, shape, creationTime);
		rsg::Box::BoxPtr resultBox = boost::dynamic_pointer_cast<rsg::Box>(shape);
		if (resultBox != 0) {
			LOG(INFO) << "Box (x,y,z) = " << resultBox->getSizeX() << " "
					<< resultBox->getSizeY() << " "
					<< resultBox->getSizeZ();
		}
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr boxPose;
		wmReplica->scene.getTransformForNode(*it, wmReplica->getRootNodeId(), creationTime, boxPose);
		LOG(INFO) << "Pose of box is = " << std::endl << *boxPose;
	}

	/* Get data of obstacles (spheres) */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(rsg::Attribute("shape", "Sphere"));
	wmReplica->scene.getNodes(queryAttributes, resultIds);

	for(vector<rsg::Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
		rsg::TimeStamp creationTime;
		rsg::Shape::ShapePtr shape;
		wmReplica->scene.getGeometry(*it, shape, creationTime);
		rsg::Sphere::SpherePtr resultSphere = boost::dynamic_pointer_cast<rsg::Sphere>(shape);
		if (resultSphere != 0) {
			LOG(INFO) << "Sphere (r) = " << resultSphere->getRadius();
		}
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr spherePose;
		wmReplica->scene.getTransformForNode(*it, wmReplica->getRootNodeId(), creationTime, spherePose);
		LOG(INFO) << "Pose of sphere is = " << std::endl << *spherePose;
	}


	/* =============== Sample updates to the world model =================== */


	/* update the pose of the second sphere */
	brics_3d::IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transformShpere2Update(new brics_3d::HomogeneousMatrix44(1,0,0,  	// Rotation coefficients
	                                                             0,1,0,
	                                                             0,0,1,
	                                                             0.9,1.55,0)); 						// Translation coefficients
	wm->scene.setTransform(sphere2TfId, transformShpere2Update, wm->now());

	/* update fence geometry be creation of a new geometry */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(rsg::Attribute("name", "virtual_fence"));
	wm->scene.getNodes(queryAttributes, resultIds);
	for(vector<rsg::Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // delete all node  with "name", "virtual_fence"
		wm->scene.deleteNode(*it);
	}
	attributes.clear();
	attributes.push_back(rsg::Attribute("shape", "Box"));
	attributes.push_back(rsg::Attribute("name", "virtual_fence"));
	rsg::Box::BoxPtr newBox(new rsg::Box(1.5,2.5,0));
	rsg::Id newBoxId;
	wm->scene.addGeometricNode(boxTfId, newBoxId, attributes, newBox, wm->now());


	/* ====== Sample queries by the motion controller after update ========= */

	/* Get data and pose of the box */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(rsg::Attribute("name", "virtual_fence"));
	wmReplica->scene.getNodes(queryAttributes, resultIds);

	for(vector<rsg::Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
		rsg::TimeStamp creationTime;
		rsg::Shape::ShapePtr shape;
		wmReplica->scene.getGeometry(*it, shape, creationTime);
		rsg::Box::BoxPtr resultBox = boost::dynamic_pointer_cast<rsg::Box>(shape);
		if (resultBox != 0) {
			LOG(INFO) << "Box (x,y,z) = " << resultBox->getSizeX() << " "
					<< resultBox->getSizeY() << " "
					<< resultBox->getSizeZ();
		}
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr boxPose;
		wmReplica->scene.getTransformForNode(*it, wmReplica->getRootNodeId(), creationTime, boxPose);
		LOG(INFO) << "Pose of box is = " << std::endl << *boxPose;
	}

	/* Get data of obstacles (spheres) */
	queryAttributes.clear();
	resultIds.clear();
	queryAttributes.push_back(rsg::Attribute("shape", "Sphere"));
	wmReplica->scene.getNodes(queryAttributes, resultIds);

	for(vector<rsg::Id>::iterator it = resultIds.begin(); it!=resultIds.end(); ++it) { // loop over results
		rsg::TimeStamp creationTime;
		rsg::Shape::ShapePtr shape;
		wmReplica->scene.getGeometry(*it, shape, creationTime);
		rsg::Sphere::SpherePtr resultSphere = boost::dynamic_pointer_cast<rsg::Sphere>(shape);
		if (resultSphere != 0) {
			LOG(INFO) << "Sphere (r) = " << resultSphere->getRadius();
		}
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr spherePose;
		wmReplica->scene.getTransformForNode(*it, wmReplica->getRootNodeId(), creationTime, spherePose);
		LOG(INFO) << "Pose of sphere is = " << std::endl << *spherePose;
	}


	/* Wait until user closes the GUI */
	while(!wm3DVisualizer->done()) {
		//nothing here
	}

	/* Clean up */
	delete wmUpdatesToHdf5Serializer;
	delete wmUpdatesToHdf5deserializer;
	delete feedForwardBridge;
	delete wm3DVisualizer;
	delete wmStructureVisualizer;
	delete wmReplicaStructureVisualizer;
	delete wm;
	delete wmReplica;
}


/* EOF */
