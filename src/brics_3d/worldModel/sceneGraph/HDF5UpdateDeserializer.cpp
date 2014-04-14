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

#include "HDF5UpdateDeserializer.h"
#include <fstream>

namespace brics_3d {
namespace rsg {

// helper struct to memorize index and name verctor
typedef struct {
    int index;               // index of the current object
    std::vector<std::string> collectedGroupNames;
} group_name_iter_info;

herr_t collectGroupNames(hid_t loc_id, const char *name, void *opdata) {
	LOG(DEBUG) << "H5::Group name = " << name;

	group_name_iter_info *info=(group_name_iter_info *)opdata;
    int obj_type = H5Gget_objtype_by_idx(loc_id, info->index);
    if(obj_type == H5G_GROUP) {
    	 LOG(DEBUG)  << "        is a group";
    		info->collectedGroupNames.push_back(name);

    } else {
		LOG(DEBUG) << "       is NOT a proup but of type: " << obj_type;
	}

    (info->index)++;
    return 0;
}

void collectAttributeNames(H5::H5Object& loc,
		const H5std_string attr_name, void *operator_data) {

	LOG(DEBUG) << "H5::Attribute name = " << attr_name;
	vector<std::string>* attributeNames = (vector<std::string>*)operator_data;
	attributeNames->push_back(attr_name);
}

HDF5UpdateDeserializer::HDF5UpdateDeserializer(WorldModel* wm) :
		wm(wm) {

}

HDF5UpdateDeserializer::~HDF5UpdateDeserializer() {

}

int HDF5UpdateDeserializer::write(const char* dataBuffer, int dataLength,
		int& transferredBytes) {

	bool result = handleSceneGraphUpdate(dataBuffer, dataLength, transferredBytes);
	return result;

}

bool HDF5UpdateDeserializer::handleSceneGraphUpdate(const char* dataBuffer,
		int dataLength, int& transferredBytes) {

	LOG(DEBUG) << "HDF5UpdateDeserializer: Receiving a new data stream.";

	/* safe as tmp file  (FIXME) */
	std::string messageName = "tmpMessage.h5";
	std::ofstream outputFile(messageName.c_str(), std::ios::binary | std::ios::trunc);
	outputFile.write(dataBuffer, dataLength);
	outputFile.flush();
	outputFile.close();

	/* open with HDF5 */
    try {
       H5::Exception::dontPrint();
       H5::H5File file(messageName, H5F_ACC_RDONLY); // re-open that tmp file

       H5::Group scene = file.openGroup("Scene");
       HDF5Typecaster::RsgUpdateCommand command;
       HDF5Typecaster::RsgNodeTypeInfo type;
       HDF5Typecaster::getCommandTypeInfoFromHDF5Group(command, scene);

       /* Discover the attached HDF5 groups */
       group_name_iter_info groupNamesIterator;
       groupNamesIterator.index = 0;
//       vector<std::string> groupNames;
       int returnValue = scene.iterateElems(".", NULL, collectGroupNames, &groupNamesIterator);
       std::vector<std::string> groupNames = groupNamesIterator.collectedGroupNames;
       if(groupNames.size() != 1) {
    	   LOG(ERROR) << "HDF5UpdateDeserializer:  Discovered " << groupNames.size() << " H5::Groups underneath Scene Group. Should be one.";
    	   file.close();
    	   return false;
       }
       std::string groupName = groupNames[0];
       H5::Group group = file.openGroup(groupName);

       /* parse incoming data */
       switch (command) {

       case HDF5Typecaster::ADD:

    	   LOG(DEBUG) << "HDF5UpdateDeserializer: Processing ADD command";
    	   HDF5Typecaster::getNodeTypeInfoFromHDF5Group(type, group);

    	   switch (type) {
    	   case HDF5Typecaster::NODE:
    		   doAddNode(group);
    		   break;

    	   case HDF5Typecaster::GROUP:
    		   doAddGroup(group);
    		   break;

    	   case HDF5Typecaster::GEOMETIRC_NODE:
    		   doAddGeometricNode(group);
    		   break;

    	   case HDF5Typecaster::TRANSFORM:
    		   doAddTransformNode(group);
    		   break;

    	   default:
    		   LOG(WARNING) << "HDF5UpdateDeserializer: Unhandeled node type: " << type;
    		   break;
    	   }

    	   break;
    	   default:
    		   LOG(WARNING) << "HDF5UpdateDeserializer: Unhandeled command: " << command;
    		   break;
       }

       /* close file */
//       file.flush(H5F_SCOPE_GLOBAL);
       file.close();

    } catch (H5::Exception e) {
    	LOG(ERROR) << "HDF5UpdateDeserializer: cannot process HDF5 file " << messageName;
    	return false;
    }

    transferredBytes = dataLength;
	return true;
}

bool HDF5UpdateDeserializer::doAddNode(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doAddGroup(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doAddTransformNode(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doAddGeometricNode(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doSetNodeAttributes(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doSetTransform(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doDeleteNode(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doAddParent(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doRemoveParent(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: functionality not yet implemented.";
	return false;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
