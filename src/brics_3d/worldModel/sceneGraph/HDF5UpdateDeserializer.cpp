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
#include "brics_3d/core/HomogeneousMatrix44.h"
#include <fstream>
#include "H5LTpublic.h" // not in default installation -> need to enable HDF5_BUILD_HLIB


namespace brics_3d {
namespace rsg {

// helper struct to memorize index and name vector
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

	/* safe as tmp file  */
	std::string messageName = "tmpMessage.h5";
//	std::ofstream outputFile(messageName.c_str(), std::ios::binary | std::ios::trunc);
//	outputFile.write(dataBuffer, dataLength);
//	outputFile.flush();
//	outputFile.close();

	/* open with HDF5 */
    try {


    	/* Create HDF5 (in-memory) image file from data buffer */
    	unsigned flags;
//    	flags = H5LT_FILE_IMAGE_DONT_COPY | H5LT_FILE_IMAGE_DONT_RELEASE;
    	flags = H5LT_FILE_IMAGE_DONT_RELEASE;
    	LOG(DEBUG) << "HDF5UpdateDeserializer: trying H5LTopen_file_image.";
    	hid_t fileId = H5LTopen_file_image((char* )dataBuffer, dataLength, flags); // to copy or not?
    	LOG(DEBUG) << "HDF5UpdateDeserializer: H5LTopen_file_image done.";
    	H5::H5File file;
    	file.setId(fileId);
    	LOG(DEBUG) << "HDF5UpdateDeserializer: HDF5 file set up.";

       //H5::Exception::dontPrint();
//    	H5::H5File file(messageName, H5F_ACC_RDONLY); // re-open that tmp file

       H5::Group scene = file.openGroup("Scene");
       LOG(DEBUG) << "HDF5UpdateDeserializer: Opened \"Scene\" group.";
       HDF5Typecaster::RsgUpdateCommand command;
       HDF5Typecaster::RsgNodeTypeInfo type;
       HDF5Typecaster::getCommandTypeInfoFromHDF5Group(command, scene);
       bool hasParentId = false;;

       try {
    	   HDF5Typecaster::getNodeIdFromHDF5Group(parentId, scene, rsgParentIdName); // we memorize the parent Id as backtrack in HDF5 is not so easy.
    	   hasParentId = true;
       } catch (H5::Exception e) {
    	   LOG(WARNING) << "No parentId given.";
       }

       /* Discover the attached HDF5 groups */
       group_name_iter_info groupNamesIterator;
       groupNamesIterator.index = 0;
       scene.iterateElems(".", NULL, collectGroupNames, &groupNamesIterator);
       std::vector<std::string> groupNames = groupNamesIterator.collectedGroupNames;
       if(groupNames.size() != 1) {
    	   LOG(ERROR) << "HDF5UpdateDeserializer:  Discovered " << groupNames.size() << " H5::Groups underneath Scene Group. Should be one.";
    	   file.close();
    	   return false;
       }
       std::string groupName = groupNames[0];
       LOG(DEBUG) << "Opening H5::Group with name " << groupName;
       H5::Group group = scene.openGroup(groupName);

       /* parse incoming data */
       switch (command) {

		   case HDF5Typecaster::ADD:

			   LOG(DEBUG) << "HDF5UpdateDeserializer: Processing ADD command";
			   if(!hasParentId) {
				   LOG(ERROR) << "Retrieved an ADD command, but without parentId - this does not work.";
				   break;

			   }
			   HDF5Typecaster::getNodeTypeInfoFromHDF5Group(type, group);

			   switch (type) {
			   case HDF5Typecaster::NODE:
				   doAddNode(group);
				   break;

			   case HDF5Typecaster::GROUP:
				   doAddGroup(group);
				   break;

			   case HDF5Typecaster::GEOMETIRC_NODE:
				   LOG(DEBUG) << "entering doAddGeometricNode(group)";
				   doAddGeometricNode(group);
				   break;

			   case HDF5Typecaster::TRANSFORM:
				   doAddTransformNode(group);
				   break;

			   default:
				   LOG(WARNING) << "HDF5UpdateDeserializer: Unhandled node type: " << type;
				   break;
			   }
			   break;

           case HDF5Typecaster::SET_TRANSFORM:

        	   LOG(DEBUG) << "HDF5UpdateDeserializer: Processing SET_TRANSFOR command";
        	   HDF5Typecaster::getNodeTypeInfoFromHDF5Group(type, group);
        	   if(type != HDF5Typecaster::TRANSFORM) {
        		   LOG(ERROR) << "Received a SET_TRANSFORM command, but node type is not a Transform.";
        		   return false;
        	   }
        	   doSetTransform(group);

        	   break;

           case HDF5Typecaster::DELETE:

        	   LOG(DEBUG) << "HDF5UpdateDeserializer: Processing DELETE command";
        	   doDeleteNode(group);

        	   break;

    	   break;
    	   default:
    		   LOG(WARNING) << "HDF5UpdateDeserializer: Unhandled command: " << command;
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
	LOG(ERROR) << "HDF5UpdateDeserializer: doAddNode functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doAddGroup(H5::Group& group) {
	LOG(DEBUG) << "HDF5UpdateDeserializer: doAddGroup.";
	Id id = 0;
	vector<Attribute> attributes;

	if (!HDF5Typecaster::getNodeIdFromHDF5Group(id, group)) {
		LOG(ERROR) << "H5::Group has no ID";
		return false;
	}
	LOG(DEBUG) << "H5::Group has ID " << id;
	HDF5Typecaster::getAttributesFromHDF5Group(attributes, group);

//	LOG(DEBUG) << "Complete group name = " << group.getObjnameByIdx(group.getId());

	return wm->scene.addGroup(parentId, id, attributes, true);
}

bool HDF5UpdateDeserializer::doAddTransformNode(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: doAddTransformNode.";

	Id id = 0;
	vector<Attribute> attributes;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44());
	TimeStamp timeStamp;

	if (!HDF5Typecaster::getNodeIdFromHDF5Group(id, group)) {
		LOG(ERROR) << "H5::Group has no ID";
		return false;
	}
	LOG(DEBUG) << "H5::Group has ID " << id;
	HDF5Typecaster::getAttributesFromHDF5Group(attributes, group);

	if(!HDF5Typecaster::getTransformFromHDF5Group(transform, timeStamp, group)) {
		LOG(ERROR) << "H5::Group has no transform dataset.";
		return false;
	}
	LOG(DEBUG) << "Transform @t " << timeStamp.getSeconds() <<"  is" << std::endl << *transform;

	return wm->scene.addTransformNode(parentId, id, attributes, transform, timeStamp, true);
}

bool HDF5UpdateDeserializer::doAddGeometricNode(H5::Group& group) {
	LOG(INFO) << "HDF5UpdateDeserializer: doAddGeometricNode.";

	Id id = 0;
	vector<Attribute> attributes;
	Shape::ShapePtr shape;
	TimeStamp timeStamp;

	if (!HDF5Typecaster::getNodeIdFromHDF5Group(id, group)) {
		LOG(ERROR) << "H5::Group has no ID";
		return false;
	}
	LOG(DEBUG) << "H5::Group has ID " << id;
	HDF5Typecaster::getAttributesFromHDF5Group(attributes, group);

	if(!brics_3d::rsg::HDF5Typecaster::getShapeFromHDF5Group(shape, group)) {
		LOG(ERROR) << "H5::Group has no Shape";
		return false;
	}

	if(!brics_3d::rsg::HDF5Typecaster::getTimeStampFromHDF5Group(timeStamp, group)) {
		LOG(ERROR) << "H5::Group has no TimeStamp";
		return false;
	}

	return wm->scene.addGeometricNode(parentId, id, attributes, shape, timeStamp, true);
}

bool HDF5UpdateDeserializer::doSetNodeAttributes(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: doSetNodeAttributes functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doSetTransform(H5::Group& group) {
	LOG(DEBUG) << "HDF5UpdateDeserializer: doSetTransform.";

	Id id;
	IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44());
	TimeStamp timeStamp;

	if (!HDF5Typecaster::getNodeIdFromHDF5Group(id, group)) {
		LOG(ERROR) << "H5::Group has no ID";
		return false;
	}

	if(!HDF5Typecaster::getTransformFromHDF5Group(transform, timeStamp, group)) {
		LOG(ERROR) << "H5::Group has no transform dataset.";
		return false;
	}

	return wm->scene.setTransform(id, transform, timeStamp);
}

bool HDF5UpdateDeserializer::doDeleteNode(H5::Group& group) {
	LOG(DEBUG) << "HDF5UpdateDeserializer: doDeleteNode.";

	Id id;
	if (!HDF5Typecaster::getNodeIdFromHDF5Group(id, group)) {
		LOG(ERROR) << "H5::Group that shall be deleted has no ID";
		return false;
	}

	return wm->scene.deleteNode(id);
}

bool HDF5UpdateDeserializer::doAddParent(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: doAddParent functionality not yet implemented.";
	return false;
}

bool HDF5UpdateDeserializer::doRemoveParent(H5::Group& group) {
	LOG(ERROR) << "HDF5UpdateDeserializer: doRemoveParent functionality not yet implemented.";
	return false;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
