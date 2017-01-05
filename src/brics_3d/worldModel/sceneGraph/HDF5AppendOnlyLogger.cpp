/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2017, KU Leuven
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

#include "HDF5AppendOnlyLogger.h"
#include "brics_3d/core/Version.h"
#include <fstream>
#include <iomanip>

namespace brics_3d {
namespace rsg {


HDF5AppendOnlyLogger::HDF5AppendOnlyLogger(WorldModel* wm) : wm(wm) {
	logFile = 0;

	std::stringstream instanceBasedSuffix;
	instanceBasedSuffix << wm->getRootNodeId().toString();
	fileSuffix = "-" + instanceBasedSuffix.str() + ".rsg.h5";

	std::stringstream tmpFileName;
	time_t rawtime;
	struct tm* timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	tmpFileName << "20" // This will have to be adjusted in year 2100 ;-)
			<< (timeinfo->tm_year)-100 << "-"
			<< std::setw(2) << std::setfill('0')
			<<	(timeinfo->tm_mon)+1 << "-"
			<< std::setw(2) << std::setfill('0')
			<<	timeinfo->tm_mday << "_"
			<< std::setw(2) << std::setfill('0')
			<<	timeinfo->tm_hour << "-"
			<< std::setw(2) << std::setfill('0')
			<<	timeinfo->tm_min << "-"
			<< std::setw(2) << std::setfill('0')
			<<	timeinfo->tm_sec;

	std::string fileName = "rsg-log-" + tmpFileName.str() + fileSuffix;

	initialize(fileName);
}

HDF5AppendOnlyLogger::HDF5AppendOnlyLogger(WorldModel* wm, string logFileName) : wm(wm) {
	logFile = 0;
	initialize(logFileName);
}

void HDF5AppendOnlyLogger::initialize(string logFileName) {

	/* some default values */
	fileImageIncremet = 1*sizeof(char);

	H5::FileAccPropList faplCore;
	faplCore.setCore(fileImageIncremet, true); // toggle in-memory behavior

	try {
		logFile = new H5::H5File (logFileName, /*H5F_ACC_RDWR | H5F_ACC_CREAT*/ H5F_ACC_TRUNC, H5::FileCreatPropList::DEFAULT, faplCore);
		LOG(INFO) << "HDF5AppendOnlyLogger: Created HDF5 log file: " << logFileName;

		vector<Attribute> logAttributes;
		logAttributes.push_back(Attribute("name","RSG HDF5 log file."));
		logAttributes.push_back(Attribute("lib_version", Version::getVersionAsString()));
		logAttributes.push_back(Attribute("log_version", "1.0.0"));
		logAttributes.push_back(Attribute("rootId", wm->getRootNodeId().toString()));
		std::stringstream logLevel;
		logLevel << brics_3d::Logger::getMinLoglevel();
		logAttributes.push_back(Attribute("log_level", logLevel.str()));
		std::stringstream stamp;
		stamp << std::fixed << wm->now().getSeconds();
		logAttributes.push_back(Attribute("startStamp", stamp.str()));
		H5::Group logMetadata = logFile->createGroup("log-metadata");
		HDF5Typecaster::addAttributesToHDF5Group(logAttributes, logMetadata);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger: Cannot create HDF5 log file";
		logFile = 0;
	}
}

HDF5AppendOnlyLogger::~HDF5AppendOnlyLogger() {
	if(logFile) {
		logFile->close();
		delete logFile;
	}
}

bool HDF5AppendOnlyLogger::addNode(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {
	LOG(DEBUG) << "HDF5AppendOnlyLogger: adding a Node-" << assignedId.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::ADD, scene);
		HDF5Typecaster::addNodeIdToHDF5Group(parentId, scene, rsgParentIdName);

		H5::Group group = scene.createGroup("Node-" + assignedId.toString()); // The actual data
		HDF5Typecaster::addNodeTypeInfoToHDF5Group(HDF5Typecaster::NODE, group);
		HDF5Typecaster::addNodeIdToHDF5Group(assignedId, group);
		HDF5Typecaster::addAttributesToHDF5Group(attributes, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger addNode: Cannot log to HDF.";
	}

	return true;
}

bool HDF5AppendOnlyLogger::addGroup(Id parentId, Id& assignedId,
		vector<Attribute> attributes, bool forcedId) {
	LOG(DEBUG) << "HDF5AppendOnlyLogger: adding a Group-" << assignedId.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::ADD, scene);
		HDF5Typecaster::addNodeIdToHDF5Group(parentId, scene, rsgParentIdName);

		H5::Group group = scene.createGroup("Group-" + assignedId.toString()); // The actual data
		HDF5Typecaster::addNodeTypeInfoToHDF5Group(HDF5Typecaster::GROUP, group);
		HDF5Typecaster::addNodeIdToHDF5Group(assignedId, group);
		HDF5Typecaster::addAttributesToHDF5Group(attributes, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger addGroup: Cannot log to HDF.";
		return false;
	}

	return true;
}

bool HDF5AppendOnlyLogger::addTransformNode(Id parentId,
		Id& assignedId, vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp, bool forcedId) {

	LOG(DEBUG) << "HDF5AppendOnlyLogger: adding a Transform-" << assignedId.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::ADD, scene);
		HDF5Typecaster::addNodeIdToHDF5Group(parentId, scene, rsgParentIdName);

		H5::Group group = scene.createGroup("Transform-" + assignedId.toString()); // The actual data
		HDF5Typecaster::addNodeTypeInfoToHDF5Group(HDF5Typecaster::TRANSFORM, group);
		HDF5Typecaster::addNodeIdToHDF5Group(assignedId, group);
		HDF5Typecaster::addAttributesToHDF5Group(attributes, group);
		HDF5Typecaster::addTransformToHDF5Group(transform, timeStamp, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger addTransformNode: Cannot log to HDF.";
		return false;
	}
	return true;
}

bool HDF5AppendOnlyLogger::addUncertainTransformNode(Id parentId,
		Id& assignedId, vector<Attribute> attributes,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp, bool forcedId) {

	LOG(DEBUG) << "HDF5AppendOnlyLogger: adding a UncertainTransform-" << assignedId.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::ADD, scene);
		HDF5Typecaster::addNodeIdToHDF5Group(parentId, scene, rsgParentIdName);

		H5::Group group = scene.createGroup("UncertainTransform-" + assignedId.toString()); // The actual data
		HDF5Typecaster::addNodeTypeInfoToHDF5Group(HDF5Typecaster::UNCERTAIN_TRANSFORM, group);
		HDF5Typecaster::addNodeIdToHDF5Group(assignedId, group);
		HDF5Typecaster::addAttributesToHDF5Group(attributes, group);
		HDF5Typecaster::addTransformToHDF5Group(transform, timeStamp, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger addTransformNode: Cannot log to HDF.";
		return false;
	}
	return true;

	return false;
}

bool HDF5AppendOnlyLogger::addGeometricNode(Id parentId,
		Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape,
		TimeStamp timeStamp, bool forcedId) {

	LOG(DEBUG) << "HDF5AppendOnlyLogger: adding a GeometricNode-" << assignedId.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::ADD, scene);
		HDF5Typecaster::addNodeIdToHDF5Group(parentId, scene, rsgParentIdName);

		H5::Group group = scene.createGroup("GeometricNode-" + assignedId.toString()); // The actual data
		HDF5Typecaster::addNodeTypeInfoToHDF5Group(HDF5Typecaster::GEOMETIRC_NODE, group);
		HDF5Typecaster::addNodeIdToHDF5Group(assignedId, group);
		HDF5Typecaster::addAttributesToHDF5Group(attributes, group);
		HDF5Typecaster::addShapeToHDF5Group(shape, group);
		HDF5Typecaster::addTimeStampToHDF5Group(timeStamp, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger addTransformNode: Cannot create a HDF serialization.";
		return false;
	}
	return true;
}

bool HDF5AppendOnlyLogger::addRemoteRootNode(Id rootId, vector<Attribute> attributes) {
	LOG(DEBUG) << "HDF5AppendOnlyLogger: adding a RemoteNode-" << rootId.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::ADD_REMOTE_ROOT_NODE, scene);

		H5::Group group = scene.createGroup("RemoteRootNode-" + rootId.toString()); // The actual data
		HDF5Typecaster::addNodeIdToHDF5Group(rootId, group);
		HDF5Typecaster::addAttributesToHDF5Group(attributes, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger addRemoteRootNode: Cannot log to HDF.";
		return false;
	}

	return true;
}

bool HDF5AppendOnlyLogger::addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId) {
	LOG(DEBUG) << "HDF5AppendOnlyLogger: adding a Connection-" << assignedId.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::ADD, scene);
		HDF5Typecaster::addNodeIdToHDF5Group(parentId, scene, rsgParentIdName);

		H5::Group group = scene.createGroup("Connection-" + assignedId.toString()); // The actual data
		HDF5Typecaster::addNodeTypeInfoToHDF5Group(HDF5Typecaster::CONNECTION, group);
		HDF5Typecaster::addNodeIdToHDF5Group(assignedId, group);
		HDF5Typecaster::addAttributesToHDF5Group(attributes, group);

		/* Connection specific payload: Id sets and time stamps */
		HDF5Typecaster::addIdsToHDF5Group(sourceIds, group, "sourceIds");
		HDF5Typecaster::addIdsToHDF5Group(targetIds, group, "targetIds");
		HDF5Typecaster::addTimeStampToHDF5Group(start, group, "start");
		HDF5Typecaster::addTimeStampToHDF5Group(end, group, "end");

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger addConnection: Cannot log to HDF.";
		return false;
	}

	return true;

}


bool HDF5AppendOnlyLogger::setNodeAttributes(Id id,
		vector<Attribute> newAttributes,  TimeStamp timeStamp) {

	LOG(DEBUG) << "HDF5AppendOnlyLogger: updating Attributes for node " << id.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::SET_ATTRIBUTES, scene);

		H5::Group group = scene.createGroup("Attribute-Update-" + id.toString()); // The actual data
		HDF5Typecaster::addNodeIdToHDF5Group(id, group);
		HDF5Typecaster::addAttributesToHDF5Group(newAttributes, group);
		HDF5Typecaster::addTimeStampToHDF5Group(timeStamp, group, "attributesTimeStamp");

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger setNodeAttributes: Cannot: Cannot log to HDF.";
	}

	return true;
}

bool HDF5AppendOnlyLogger::setTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		TimeStamp timeStamp) {

	LOG(DEBUG) << "HDF5AppendOnlyLogger: updating a Transform-" << id.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::SET_TRANSFORM, scene);

		H5::Group group = scene.createGroup("Transform-Update-" + id.toString()); // The actual data
		HDF5Typecaster::addNodeTypeInfoToHDF5Group(HDF5Typecaster::TRANSFORM, group);
		HDF5Typecaster::addNodeIdToHDF5Group(id, group);
		HDF5Typecaster::addTransformToHDF5Group(transform, timeStamp, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger setTransform: Cannot Cannot log to HDF.";
		return false;
	}

	return true;
}

bool HDF5AppendOnlyLogger::setUncertainTransform(Id id,
		IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform,
		ITransformUncertainty::ITransformUncertaintyPtr uncertainty,
		TimeStamp timeStamp) {

	LOG(DEBUG) << "HDF5AppendOnlyLogger: updating an UncertainTransform-" << id.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::SET_TRANSFORM, scene);

		H5::Group group = scene.createGroup("Transform-Update-" + id.toString()); // The actual data
		HDF5Typecaster::addNodeTypeInfoToHDF5Group(HDF5Typecaster::UNCERTAIN_TRANSFORM, group);
		HDF5Typecaster::addNodeIdToHDF5Group(id, group);
		HDF5Typecaster::addTransformToHDF5Group(transform, timeStamp, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger setUncertainTransform: Cannot Cannot log to HDF.";
		return false;
	}

	return true;
}

bool HDF5AppendOnlyLogger::deleteNode(Id id) {
	LOG(DEBUG) << "HDF5AppendOnlyLogger: deleting Node-" << id.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::DELETE, scene);

		H5::Group group = scene.createGroup("Node-" + id.toString()); // The actual data
		HDF5Typecaster::addNodeIdToHDF5Group(id, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger setTransform: Cannot log to HDF.";
		return false;
	}

	return true;
}

bool HDF5AppendOnlyLogger::addParent(Id id, Id parentId) {
	LOG(DEBUG) << "HDF5AppendOnlyLogger: adding Parent " << parentId.toString() << " to Node " << id.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::ADD_PARENT, scene);
		HDF5Typecaster::addNodeIdToHDF5Group(parentId, scene, rsgParentIdName);

		H5::Group group = scene.createGroup("Parent-Child-Relation-" + id.toString()); // The actual data
		HDF5Typecaster::addNodeIdToHDF5Group(id, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger addParent: Cannot log to HDF.";
		return false;
	}

	return true;
}

bool HDF5AppendOnlyLogger::removeParent(Id id, Id parentId) {
	LOG(DEBUG) << "HDF5AppendOnlyLogger: removing Parent " << parentId.toString() << " to Node " << id.toString();
	try {

		/* Generic/common entry group with general information */
		std::stringstream groupName;
		groupName << std::fixed << wm->now().getSeconds();
		H5::Group scene = logFile->createGroup(groupName.str());
		HDF5Typecaster::addCommandTypeInfoToHDF5Group(HDF5Typecaster::REMOVE_PARENT, scene);
		HDF5Typecaster::addNodeIdToHDF5Group(parentId, scene, rsgParentIdName);

		H5::Group group = scene.createGroup("Parent-Child-Relation-" + id.toString()); // The actual data
		HDF5Typecaster::addNodeIdToHDF5Group(id, group);

		logFile->flush(H5F_SCOPE_GLOBAL);

	} catch (H5::Exception e) {
		LOG(ERROR) << "HDF5AppendOnlyLogger addParent: Cannot log to HDF.";
		return false;
	}
	return true;
}



} /* namespace rsg */
} /* namespace brics_3d */
/* EOF */
