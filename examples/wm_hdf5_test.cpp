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

/* BRICS_3D includes */

#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/core/PointCloud3D.h>
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateSerializer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateDeserializer.h>

#include <brics_3d/util/HDF5Typecaster.h>

using namespace brics_3d;
using brics_3d::Logger;

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

// "Is" an output port and "has" an input port to directly feed forward data (byte array).
class HSDF5SimleBridge : public brics_3d::rsg::IOutputPort {
public:
	HSDF5SimleBridge(brics_3d::rsg::IInputPort* inputPort) : inputPort(inputPort){};
	virtual ~HSDF5SimleBridge(){};

	int write(const char *dataBuffer, int dataLength, int &transferredBytes) {
		LOG(INFO) << "HSDF5SimleBridge: Feeding data forwards.";
//		hexdump(dataBuffer, dataLength);
		return inputPort->write(dataBuffer, dataLength, transferredBytes); // just feed forward
	};

private:
	brics_3d::rsg::IInputPort* inputPort;
};

class OutputPortDummy : public brics_3d::rsg::IOutputPort {
public:
	OutputPortDummy(){};
	virtual ~OutputPortDummy(){};
	int write(const char *dataBuffer, int dataLength, int &transferredBytes){
		hexdump(dataBuffer, dataLength);
		transferredBytes = dataLength;
		return 0;
	};
};


int main(int argc, char **argv) {

	std::string dataSetFolder = BRICS_MODELS_DIR;
	std::string dataSet = dataSetFolder + "/bunny000.txt";

	/* Configure logger - default level won't tell us much */
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	/* Create WM  handles */
	brics_3d::WorldModel* wm = new brics_3d::WorldModel();
	brics_3d::WorldModel* wmReplica = new brics_3d::WorldModel();

	/* Attach some visualization facilities */
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
	 * Connect both world models via:
	 * some update on first world model -> call observer class -> serialize
	 * -> send byte steam via HSDF5SimleBridge -> deserialize -> update on second world model
	 */
	brics_3d::rsg::HDF5UpdateDeserializer* wmUpdatesToHdf5deserializer = new brics_3d::rsg::HDF5UpdateDeserializer(wmReplica);
	//OutputPortDummy port;
	HSDF5SimleBridge feedWorwardBridge(wmUpdatesToHdf5deserializer);
	brics_3d::rsg::HDF5UpdateSerializer* wmUpdatesToHdf5Serializer = new brics_3d::rsg::HDF5UpdateSerializer(&feedWorwardBridge);
	wm->scene.attachUpdateObserver(wmUpdatesToHdf5Serializer);

	/* Load a data set */
	brics_3d::PointCloud3D::PointCloud3DPtr pointCloud1(new brics_3d::PointCloud3D());
	brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr pointCloud1Container(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());
	pointCloud1Container->data = pointCloud1;
	pointCloud1Container->data->readFromTxtFile(dataSet);

	/* Add point cloud to wm */
    brics_3d::rsg::Id pointCloud1Id = 0;
    std::vector<brics_3d::rsg::Attribute> attributes;
    attributes.clear();
    attributes.push_back(brics_3d::rsg::Attribute("name","pointCloud1Id"));
	wm->scene.addGeometricNode(wm->getRootNodeId(), pointCloud1Id, attributes, pointCloud1Container, wm->now());


	/* Add group nodes */
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





	LOG(INFO) << "Done so far.";

	/* Wait until user closes the GUI */
	while(!wm3DVisualizer->done()) {
		//nothing here
	}

	/* Clean up */
	delete wmUpdatesToHdf5Serializer;
	delete wm3DVisualizer;
	delete wmStructureVisualizer;
	delete wm;
	delete wmReplica;
}


/* EOF */
