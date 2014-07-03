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
 * Both instances have to be started in seperate processes.
 * I.e. _this_ test binary has to be started twice.
 * The process that is started first will recieve all updates
 * from the other (while not necesarrily vice versa)
 *
 *  +----------+                +-------------+
 *  |    wm    |  <---UDP--->   |     wm      |
 *  +----------+    updates     +-------------+
 *
 * NOTE: Start the processes from diffrent relative locations
 * such that the generated dabug output files do not intermix.
 */

/* BRICS_3D includes */
#include <brics_3d/core/Logger.h>
#include <brics_3d/core/HomogeneousMatrix44.h>
#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/DotVisualizer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateSerializer.h>
#include <brics_3d/worldModel/sceneGraph/HDF5UpdateDeserializer.h>
#ifdef BRICS_OSG_ENABLE
	#include <brics_3d/worldModel/sceneGraph/OSGVisualizer.h>
#endif

#include <brics_3d/util/HDF5Typecaster.h>
#include <hdf5.h>

/* UDP includes*/
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>


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

/**
 * Implementation of OUT data transmission.
 *
 * A serialized UDF "messages" is send via UPD.
 */
class HSDF5UDPOutputBridge : public brics_3d::rsg::IOutputPort {
public:
	HSDF5UDPOutputBridge(std::string host, int port) : debugTag("HSDF5UDPOutputBridge") {
		std::string debugTag = "HSDF5UDPBridge";
		initialize(host, port);
		LOG(DEBUG) << debugTag << " created.";
	};

	virtual ~HSDF5UDPOutputBridge(){};

	void initialize(std::string host, int port) {

		socketDescriptor = socket (AF_INET, SOCK_DGRAM, 0);

		if (socketDescriptor == -1) {
	    	LOG(ERROR) << debugTag << "Cannot create OUT socket";
	    	return;
		} else {
			memset (&address, 0, sizeof (address));
			address.sin_family = AF_INET;
			address.sin_addr.s_addr = inet_addr (host.c_str());
			address.sin_port = htons (port);
			std::cout << debugTag <<" ready to publish";
		}

	}

	int write(const char *dataBuffer, int dataLength, int &transferredBytes) {
		LOG(DEBUG) << debugTag << ": Feeding data forwards.";
//		hexdump(dataBuffer, dataLength);

		transferredBytes = sendto(socketDescriptor,
				dataBuffer,
				static_cast<size_t>(dataLength),
				0,
				(struct sockaddr *) &address,
				sizeof (address));

		sleep(1); //just to be able to follow the output on std out...

		LOG(INFO) << debugTag << ": " << transferredBytes << " of << " << static_cast<size_t>(dataLength) << " bytes send.";

		if (transferredBytes < dataLength) {
			LOG(ERROR) << "Error in " << debugTag << " while sendind data.";
			return -1;
		}

		return 1;
	};

private:

	/// Tag that will appear in the log output
	std::string debugTag;

	/// Handle to socket
	int socketDescriptor;

	/// The IP Address
	struct sockaddr_in address;

};

/**
 * Implementation of IN data transmission.
 *
 * A worker threat listens to a UDP port.
 * Incomming data will be deseralized (HDF5) and according updates forwarded to the world model.
 */
class HSDF5UDPInputBridge {
public:
	HSDF5UDPInputBridge(brics_3d::rsg::IInputPort* inputPort, std::string host, int port) :
		inputPort(inputPort), host(host), port(port), debugTag("HSDF5UDPInputBridge") {

		LOG(DEBUG) << debugTag << " created.";
		initialize();
	};

	virtual ~HSDF5UDPInputBridge(){};

	void initialize() {
	    socketDescriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	    if (socketDescriptor == -1) {
	    	LOG(ERROR) << debugTag << "Cannot create IN socket";
	    	return;

	    } else {

	    	bzero(&hostAddress, sizeof(hostAddress));
	    	hostAddress.sin_family = AF_INET;
	    	hostAddress.sin_addr.s_addr = inet_addr (host.c_str());
	    	hostAddress.sin_port = htons(port);

	    	if (bind(socketDescriptor, (struct sockaddr* ) &hostAddress, sizeof(hostAddress))==-1) {
	    		LOG(ERROR) << "Can not bind.";
	    		return;
	    	} else {
	    		LOG(INFO) << debugTag << ": bind was successful.";
	    	}

	    	/* The worker thread handles all incomming data */
	    	pthread_create(&workerThread, NULL, _recieverThread, this);
	    }
	}

	void* recieverThread() {
	    const unsigned int BUFLEN = 20000; // For HDF5 messages longer than this we need some other transport mechanism.
	    									 // In this demo all upatates are around 10k. Thus, this buffer is sufficient.
	    char buf[BUFLEN];
	    socklen_t slen=sizeof(clientAddress);
	    ssize_t readBytes;
	    int deseraializetBytes;

	    /* Receiver loop */
	    while(true) {

	    	readBytes = recvfrom(socketDescriptor, buf, BUFLEN, 0, (struct sockaddr*)&clientAddress, &slen);

	    	if (readBytes < 0) {
	    		LOG(ERROR) << "recvfrom()";
	    		break;
	    	}

	    	LOG(INFO) << "Recieved" << readBytes << " bytes from "
	    			<< inet_ntoa(clientAddress.sin_addr) << "::"
	    			<< ntohs(clientAddress.sin_port);

	    	inputPort->write(buf, readBytes, deseraializetBytes); // feed forward to the HDF5 deserialization
	    }

	    /* Clean up */
	    close(socketDescriptor);
	    return 0;

	}

private:

	/* workaround to call the instance method */
	static void* _recieverThread(void* arg) {
	    ((HSDF5UDPInputBridge*) arg)->recieverThread();
	    return 0;
	}

private:

	/// Interface to a port that handles the un-mashaling and update of the world model
	brics_3d::rsg::IInputPort* inputPort;

	/// Name of host
	std::string host;

	/// Port that belongst to host
	int port;

	/// Tag that will appear in the log output
	std::string debugTag;

	/// Handle to socket
	int socketDescriptor;

	/// Host IP
	struct sockaddr_in hostAddress;

	/// Client IP that connects to this host
	struct sockaddr_in clientAddress;

	/// Thread that does the work
    pthread_t workerThread;

};

int main(int argc, char **argv) {

	std::string hostIP ="224.0.0.1";
	if (argc == 1) {
		LOG(INFO) << "Usage: " << argv[0] << " <host_ip>";
	} else if (argc == 2) {
		hostIP = argv[1];
	}
	LOG(INFO) << "Host IP = " << hostIP;

	/* Configure the logger - default level won't tell us much */
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::LOGDEBUG);

	/* Create the world model instances/handles */
	brics_3d::WorldModel* wm = new brics_3d::WorldModel();        // a delegate
	wm->scene.setCallObserversEvenIfErrorsOccurred(false);


	/* Attach some (optional) visualization facilities (observers) */
	brics_3d::rsg::VisualizationConfiguration visualizationConfig; // optional configuration
	visualizationConfig.visualizeIds = true;
	visualizationConfig.visualizeAttributes = false;
	visualizationConfig.abbreviateIds = true;

	brics_3d::rsg::DotVisualizer* wmStructureVisualizer = new brics_3d::rsg::DotVisualizer(&wm->scene);
	wmStructureVisualizer->setConfig(visualizationConfig); // we use the same config here
	wmStructureVisualizer->setKeepHistory(true);
	wm->scene.attachUpdateObserver(wmStructureVisualizer);

#ifdef BRICS_OSG_ENABLE
	brics_3d::rsg::OSGVisualizer* wm3DVisualizer = new brics_3d::rsg::OSGVisualizer();
	wm3DVisualizer->setConfig(visualizationConfig);
	wm->scene.attachUpdateObserver(wm3DVisualizer); //enable visualization
	usleep(500*1000); // the OSG visualization window seems to need some setup time
#endif

	/*
	 * Setup of OUT bridge (via UDP)
	 *
	 * NOTE: 224.0.0.1 is a multicast address so we don't have to bother who is server and who is client.
	 */
	HSDF5UDPOutputBridge* udpOutBridge = new HSDF5UDPOutputBridge(hostIP, 11411);
	brics_3d::rsg::HDF5UpdateSerializer* wmUpdatesToHdf5Serializer = new brics_3d::rsg::HDF5UpdateSerializer(udpOutBridge);
	wm->scene.attachUpdateObserver(wmUpdatesToHdf5Serializer);
	wmUpdatesToHdf5Serializer->setStoreMessageBackupsOnFileSystem(true); /* set to true to store all updates as .h5 files */

	/* Setup of IN bridge (via UDP) */
	brics_3d::rsg::HDF5UpdateDeserializer* wmUpdatesToHdf5deserializer = new brics_3d::rsg::HDF5UpdateDeserializer(wm);
	HSDF5UDPInputBridge* udpInBridge = new HSDF5UDPInputBridge(wmUpdatesToHdf5deserializer, "224.0.0.1", 11411);

	/* ================== Setup of world model content ===================== */

	/* Add group nodes */
	std::vector<brics_3d::rsg::Attribute> attributes;
	attributes.clear();
	attributes.push_back(rsg::Attribute("taskType", "scene_objecs"));
	pid_t pid = getpid();
	std::stringstream pidAsSteam("");
	pidAsSteam << pid;
	attributes.push_back(rsg::Attribute("PID", pidAsSteam.str()));
	rsg::Id sceneObjectsId;
	wm->scene.addGroup(wm->getRootNodeId(), sceneObjectsId, attributes);

	/* Box for "virtual fence" */
	attributes.clear();
	attributes.push_back(rsg::Attribute("name", "box_tf"));
	attributes.push_back(rsg::Attribute("taskType", "sceneObject"));
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




#ifdef BRICS_OSG_ENABLE
	/* Wait until user closes the GUI */
	while(!wm3DVisualizer->done()) {
		//nothing here
	}
#else
	while(true){}; // Continue to recieve messages.
#endif

	/* Clean up */
#ifdef BRICS_OSG_ENABLE
	delete wm3DVisualizer;
#endif
	delete wmUpdatesToHdf5Serializer;
	delete wmUpdatesToHdf5deserializer;
	delete udpOutBridge;
	delete udpInBridge;
	delete wmStructureVisualizer;
	delete wm;
}


/* EOF */
