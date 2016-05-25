/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
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

#include "WorldModel.h"
#include "brics_3d/core/Logger.h"
#include "brics_3d/core/HomogeneousMatrix44.h"

//#define BRICS_MICROBLX_ENABLE
#ifdef BRICS_MICROBLX_ENABLE
#include <ubx.h>
#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>

/* microblx types for the robot scene graph */
#include "/opt/src/sandbox/brics_3d_function_blocks/types/rsg/types/rsg_types.h" // TODO use FBX_MODULES path instead
#include "brics_3d/util/UbxTypecaster.h"

//brics_3d::WorldModel* brics_3d::WorldModel::microBlxWmHandle = 0;




#endif

using namespace brics_3d::rsg;

namespace brics_3d {

WorldModel::WorldModel() {
	this->microBlxPath = "MICROBLX_ROOT_DIR-NOT-FOUND";
	microBloxIsInitialized = false;
#ifdef BRICS_MICROBLX_ENABLE
	microBlxNodeHandle = 0;
#endif /* BRICS_MICROBLX_ENABLE */
}

void WorldModel::initializeMicroblx() {
#ifdef BRICS_MICROBLX_ENABLE
	this->microBlxPath = MICROBLX_ROOT_DIR;

	/* init microblx */
	microBlxNodeHandle = new ubx_node_info_t(); // holds all microblx
	std::string microBlxNodeName = "functionBlocks";
	LOG(DEBUG) << "WorldModel::initializeMicroblx handle with name " << microBlxNodeName << " created.";

	if (ubx_node_init(microBlxNodeHandle, microBlxNodeName.c_str()) != 0 ) { //segfaults if started from ubx lua
		LOG(ERROR) << "WorldModel::initialize: Cannot initialize the microblx node handle.";
		microBlxNodeHandle = 0;
	}
	LOG(DEBUG) << "WorldModel::initializeMicroblx ubx node created.";

	/* load the standard types */
	std::string moduleFile = microBlxPath + "std_types/stdtypes/stdtypes.so";
	if(ubx_module_load(microBlxNodeHandle, moduleFile.c_str()) != 0){
		LOG(ERROR) << "WorldModel::initialize: Cannot load the stdtypes.";
	}

	/* load the standard types */
	moduleFile = "/home/sblume/sandbox/brics_3d_function_blocks/lib/rsg_types.so";
	if(ubx_module_load(microBlxNodeHandle, moduleFile.c_str()) != 0){
		LOG(ERROR) << "WorldModel::initialize: Cannot load the rsgtypes.";
	}

	/* load a standard interaction block for sending data */
	moduleFile = microBlxPath + "std_blocks/lfds_buffers/lfds_cyclic.so";
	if(ubx_module_load(microBlxNodeHandle, moduleFile.c_str()) != 0){
		LOG(ERROR) << "WorldModel::initialize: Cannot load the lfds_buffer.";
	}

	/*
	 * Create a single (dummy) fifo interaction block to be used
	 * as default input for any function block
	 */

	/* create the input fifo block */
	std::string name = "inputFifo";
	std::string inputModuleName = "lfds_buffers/cyclic";
	if((inputBlock = ubx_block_create(microBlxNodeHandle, inputModuleName.c_str(), name.c_str())) == 0){
		LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot create the module "<< name;
		return;
	}
	LOG(DEBUG) << "WorldModel::initialize: Created block with name = " << inputBlock->name;

	/* configure the fifo block */
//	{ .name="type_name", .type_name = "char", .doc="name of registered microblx type to transport" },
//	{ .name="data_len", .type_name = "uint32_t", .doc="array length (multiplier) of data (default: 1)" },
//	{ .name="buffer_len", .type_name = "uint32_t", .doc="max. number of data elements the buffer shall hold" },
	uint32_t dataSize = sizeof(struct rsg_ids); // sizeof(uint32_t); //  sizeof(struct rsg_ids);
	uint32_t bufferSize = 4;

	ubx_data_t* fifoData = ubx_config_get_data(inputBlock, "data_len");
	memcpy(fifoData->data, &dataSize, sizeof(dataSize));

	fifoData = ubx_config_get_data(inputBlock, "buffer_len");
	memcpy(fifoData->data, &bufferSize, sizeof(bufferSize));

	fifoData = ubx_config_get_data(inputBlock, "type_name");
	int len = strlen("struct rsg_ids")+1;
	ubx_data_resize(fifoData, len);
	strncpy((char*)fifoData->data, "struct rsg_ids", len);

	/* initialize the block */
	if(ubx_block_init(inputBlock) != 0){
		LOG(ERROR) << "WorldModel::initialize: Cannot initialize the module "<< name;
		return;
	}

	/* start the block */
	if(ubx_block_start(inputBlock) != 0){
		LOG(ERROR) << "WorldModel::initialize: Cannot start the module "<< name;
		return;
	}

	/*
	 * Create a second  (dummy) fifo interaction block to be used
	 * as default output for any function block
	 */

	/* create the output fifo block */
	name = "outputFifo";
	inputModuleName = "lfds_buffers/cyclic";
	if((outputBlock = ubx_block_create(microBlxNodeHandle, inputModuleName.c_str(), name.c_str())) == 0){
		LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot create the module "<< name;
		return;
	}
	LOG(DEBUG) << "WorldModel::initialize: Created block with name = " << outputBlock->name;
	dataSize = sizeof(struct rsg_ids);
	bufferSize = 4;

	ubx_data_t* outputFifoData = ubx_config_get_data(outputBlock, "data_len");
	memcpy(outputFifoData->data, &dataSize, sizeof(dataSize));

	outputFifoData = ubx_config_get_data(outputBlock, "buffer_len");
	memcpy(outputFifoData->data, &bufferSize, sizeof(bufferSize));

	outputFifoData = ubx_config_get_data(outputBlock, "type_name");
	len = strlen("struct rsg_ids")+1;  //'rsg_ids' but should be 'struct rsg_ids'
	ubx_data_resize(outputFifoData, len);
	strncpy((char*)outputFifoData->data, "struct rsg_ids", len);

	/* initialize the block */
	if(ubx_block_init(outputBlock) != 0){
		LOG(ERROR) << "WorldModel::initialize: Cannot initialize the module "<< name;
		return;
	}

	/* start the block */
	if(ubx_block_start(outputBlock) != 0){
		LOG(ERROR) << "WorldModel::initialize: Cannot start the module "<< name;
		return;
	}

	outputBlockCopy = outputBlock;
//	WorldModel::microBlxWmHandle = this;

#endif
}

WorldModel::~WorldModel() {

#ifdef BRICS_MICROBLX_ENABLE
//	WorldModel::microBlxWmHandle = 0;
	//ubx_node_cleanup(microBlxNodeHandle); // FIXME causes segfault

#endif
	LOG(DEBUG) << "Shutting down world model. Goodbye.";
}

WorldModel::WorldModel(rsg::IIdGenerator* idGenerator) : scene(idGenerator) {
	this->microBlxPath = "MICROBLX_ROOT_DIR-NOT-FOUND";
	microBloxIsInitialized = false;
#ifdef BRICS_MICROBLX_ENABLE
	microBlxNodeHandle = 0;
#endif /* BRICS_MICROBLX_ENABLE */
}


void WorldModel::getSceneObjects(vector<rsg::Attribute> attributes, vector<SceneObject>& results) {
	TimeStamp currentTime = now();
	vector<rsg::Id>resultIds;
	results.clear();

	scene.getNodes(attributes, resultIds);
	for (unsigned int i = 0; i < static_cast<unsigned int>(resultIds.size()); ++i) {
		SceneObject tmpSceneObject;
		tmpSceneObject.id = resultIds[i];

		vector<rsg::Id> parentIds;
		scene.getNodeParents(resultIds[i], parentIds);
		if (static_cast<unsigned int>(parentIds.size()) < 1) {
			tmpSceneObject.parentId = 0; // not initialized
			LOG(WARNING) << "SceneObject has not parent node. Setting value to 0.";
		} else {
			tmpSceneObject.parentId = parentIds[0]; // here we arbitrarily select the fist parent; assumption: tree
		}

		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr tmpTransform(new HomogeneousMatrix44());
		if(!scene.getTransform(resultIds[i], currentTime, tmpTransform)) {
			LOG(WARNING) << "SceneObject with ID " << resultIds[i] << " is not a TansformNode. Skipping.";
			continue;
		}
		tmpSceneObject.transform = tmpTransform;

		vector<rsg::Id> childIds;
		scene.getGroupChildren(resultIds[i], childIds);
		for (unsigned int j = 0; j < static_cast<unsigned int>(childIds.size()); ++j) {
			Shape::ShapePtr tmpShape;
			TimeStamp dataInsertionTime;
			if (scene.getGeometry(childIds[j], tmpShape, dataInsertionTime) == true) {
				tmpSceneObject.shape = tmpShape;
				break; //stop on first found geometry
			}
		}


		vector<rsg::Attribute> tmpAttributes;
		scene.getNodeAttributes(resultIds[i], tmpAttributes);
		tmpSceneObject.attributes = tmpAttributes; //TODO combined attributes from multible nodes?

		results.push_back(tmpSceneObject);
	}
}

void WorldModel::getCurrentTransform(rsg::Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform) {

}

void WorldModel::insertTransform(rsg::Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform) {
	TimeStamp currentTime = now();
	scene.setTransform(id, transform, currentTime);
}

void WorldModel::addSceneObject(SceneObject newObject, rsg::Id& assignedId) {
	TimeStamp currentTime(timer.getCurrentTime(), Units::MilliSecond);
	rsg::Id dummyResultID;
	vector<Attribute> emptyAttributes;
	emptyAttributes.clear();

	/* a scene object is essentially a transform */
	scene.addTransformNode(scene.getRootId(), assignedId, newObject.attributes, newObject.transform, currentTime);

	/* add shape as a child node */
	scene.addGeometricNode(assignedId, dummyResultID, emptyAttributes, newObject.shape, currentTime);
}

bool WorldModel::loadFunctionBlock(std::string name) {
	/* provide a reasonable default value */
	std::string pathToBlock = microBlxPath + "std_blocks/" + name  + "/";
	return loadFunctionBlock(name, pathToBlock);
}

bool WorldModel::loadFunctionBlock(std::string name, std::string path) {
#ifdef BRICS_MICROBLX_ENABLE

	/* initialize on first load
	 * This is basically a workaround for the segfault caused by crweatin a world model
	 * via the lua bindings. In Lua you cannot call loadFunctionBlock right now...
	 */
	if (!microBloxIsInitialized) {
		initializeMicroblx();
		microBloxIsInitialized = true;
	}

	std::string moduleFile;
	ubx_block_t* block;

	LOG(DEBUG) << "WorldModel::loadFunctionBlock: Loading a new function block to the world model with name " << name;
	/* In a nutshell: you'll need to dlopen(1) the
	 * block or type library and register it with a node, the create the block
	 * instances, configure and start them.
	 */

	/* check if node is initialized */
	if (microBlxNodeHandle == 0) {
		LOG(ERROR) << "WorldModel::loadFunctionBlock: microblx node handle not initialized. Cannot load function block "<< name;
		return false;
	}

	/* load the block */
	moduleFile = path + name + ".so";
	if(ubx_module_load(microBlxNodeHandle, moduleFile.c_str()) != 0) {
		LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot load the module "<< name;
		return false;
	}

	/* create the block */
	std::string blockName = name  + "/"  + name; // e.g. "cppdemo/cppdemo"
	if((block = ubx_block_create(microBlxNodeHandle, blockName.c_str(), name.c_str())) == 0){
		LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot create the module "<< name;
		return false;
	}
	LOG(DEBUG) << "WorldModel::loadFunctionBlock: Created block with name = " << block->name;

	/* configure the block - i.e. pass the world model handle */
	//{ .name="wm_handle", .type_name = "struct rsg_wm_handle", doc="Handle to the world wodel instance. This parameter is mandatory."},
	ubx_data_t* configData = ubx_config_get_data(block, "wm_handle");
	if (configData != 0) {
		rsg_wm_handle tmpUbxWorldModleHandle; // Ubx and Lua parsable verison of a world model handle.
		tmpUbxWorldModleHandle.wm = reinterpret_cast<void*>(this);
		memcpy(configData->data, &tmpUbxWorldModleHandle, sizeof(tmpUbxWorldModleHandle));
	} else {
		LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot configure the module "<< name
				<< ", because its configuration paremeter with name wmHandle is missing.";
		return false;
	}


	/* initialize the block */
	if(ubx_block_init(block) != 0){
		LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot initialize the module "<< name;
		return false;
	}

	/* start the block */
	if(ubx_block_start(block) != 0){
		LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot start the module "<< name;
		return false;
	}

	ubx_port_t* port = ubx_port_get(block, "inputDataIds");
	if(port == 0) {
		LOG(WARNING) << "WorldModel::loadFunctionBlock: function block " << name << " has no inputDataIds port";
		// return false ?
	} else {

		/* connect to default input */
		if(ubx_port_connect_in(port, inputBlock)) {
			LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot connect the module "<< name << "to the default input port.";
			return false;
		}
	}

	ubx_port_t* outputPort = ubx_port_get(block, "outputDataIds");
	if(outputPort == 0) {
		LOG(WARNING) << "WorldModel::loadFunctionBlock: function block " << name << " has no outputDataIds port";
		// return false ?
	} else {

		/* connect to default input */
		if(ubx_port_connect_out(outputPort, outputBlock)) {
			LOG(ERROR) << "WorldModel::loadFunctionBlock: Cannot connect the module "<< name << "to the default output port.";
			return false;
		}
	}

	LOG(DEBUG) << "WorldModel::loadFunctionBlock: The are currently loaded: " << std::endl
			<< "\t" << ubx_num_blocks(microBlxNodeHandle) << " block(s)"  << std::endl
			<< "\t" << ubx_num_types(microBlxNodeHandle) << " type(s)"  << std::endl
			<< "\t" << ubx_num_modules(microBlxNodeHandle)<< " module(s)";

	LOG(INFO) << "WorldModel::loadFunctionBlock: function block " << name << " has been successfully loaded.";
	return true;

#endif
	LOG(ERROR) << "Microblx support not enabled. Cannot load a function block.";
	return false;

}

bool WorldModel::executeFunctionBlock(std::string name, std::vector<rsg::Id>& input, std::vector<rsg::Id>& output) {
#ifdef BRICS_MICROBLX_ENABLE

	/* check if node is initialized */
	if (microBlxNodeHandle == 0) {
		LOG(ERROR) << "WorldModel::executeFunctionBlock: microblx node handle not initialized. Cannot execute function block "<< name;
		return false;
	}

//	LOG(DEBUG) << "WorldModel::executeFunctionBlock: The are currently loaded: " << std::endl
//			<< "\t" << ubx_num_blocks(microBlxNodeHandle) << " block(s)"  << std::endl
//			<< "\t" << ubx_num_types(microBlxNodeHandle) << " type(s)"  << std::endl
//			<< "\t" << ubx_num_modules(microBlxNodeHandle)<< " module(s)";

	/* resolve name to block handle */
	ubx_block_t* block = ubx_block_get(microBlxNodeHandle, name.c_str());
	if (block == 0) {
		LOG(ERROR) << "WorldModel::executeFunctionBlock: function block " << name << " does not exist.";
		return false;
	}

	/* Translate inputs */
	ubx_port_t* inputPort = ubx_port_get(block, "inputDataIds");
	ubx_type_t* type =  ubx_type_get(microBlxNodeHandle, "struct rsg_ids");
	if(inputPort == 0) {
		LOG(WARNING) << "WorldModel::executeFunctionBlock: function block " << name << " has no inputDataIds port";
		//		return false;
	} else { // Only write input if it can be read...

		/* prepare rsg type */
		rsg_ids tmpIds;
//		ubx_type_t* type =  ubx_type_get(microBlxNodeHandle, "struct rsg_ids");
		brics_3d::rsg::UbxTypecaster::convertIdsToUbx(input, tmpIds); // rsg -> ubx type (structs)

		/* stuff everything into generic ubx_data_t struct */
		ubx_data_t val;
		val.type = type;
		val.len = 1;// because 1* ids struct ...  sizeof(tmpIds);//inputPointCloudId);
		val.data = &tmpIds;

		/* push data into the port */
		inputBlock->write(inputBlock, &val);
		inputBlock->stat_num_writes++;
	}

	/* Execute now! */
	if (ubx_cblock_step(block) != 0) {
		LOG(ERROR) << "WorldModel::executeFunctionBlock: cannot execute function block " << name;
		return false;
	}

	/* Translate outputs */
	output.clear();
	ubx_port_t* outputPort = ubx_port_get(block, "outputDataIds");
	if(outputPort == 0) {
		LOG(WARNING) << "WorldModel::executeFunctionBlock: function block " << name << " has no outputDataIds port";

	} else {
		rsg_ids recievedOutputDataIds;
		recievedOutputDataIds.numberOfIds = 0u;

//		int ret = read_rsg_ids(outputPort, &recievedOutputDataIds);
//		if (ret < 1) {
//			LOG(WARNING) << "WorldModel::executeFunctionBlock: No output IDs given.";
//		}

		ubx_data_t val;
		val.type = type;
		val.len = 1;// because 1* ids struct ...  sizeof(tmpIds);//inputPointCloudId);
		val.data = &recievedOutputDataIds;

		if (outputBlock != outputBlockCopy) { //workaround
			LOG(ERROR) << "Handle to output block has been corrupted."  << std::endl;
			return false;
		}
		outputBlock->read(outputBlock, &val); //can segfault
		outputBlock->stat_num_reads++;

		brics_3d::rsg::UbxTypecaster::convertIdsFromUbx(recievedOutputDataIds, output);
	}

	return true;
#endif
	LOG(ERROR) << "Microblx support not enabled. Cannot load a function block.";
	return false;
}

bool WorldModel::getLoadedFunctionBlocks(std::vector<std::string>& functionBlocks) {
#ifdef BRICS_MICROBLX_ENABLE
	LOG(INFO) << "WorldModel::executeFunctionBlock: The are currently loaded: " << std::endl
			<< "\t" << ubx_num_blocks(microBlxNodeHandle) << " block(s)"  << std::endl
			<< "\t" << ubx_num_types(microBlxNodeHandle) << " type(s)"  << std::endl
			<< "\t" << ubx_num_modules(microBlxNodeHandle)<< " module(s)";
	return false;
#endif
	return false;
}

bool WorldModel::setFunctionBlockConfiguration(std::string name, std::vector<rsg::Attribute> configuration) {
	return false;
}

rsg::Id WorldModel::getRootNodeId() {
	return scene.getRootId();
}

brics_3d::rsg::TimeStamp WorldModel::now() {
	return TimeStamp(timer.getCurrentTime(), Units::MilliSecond);
}

} // namespace brics_3d

/* EOF */

