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

#ifndef HDF5TYPECASTER_H_
#define HDF5TYPECASTER_H_

#include <brics_3d/core/Logger.h>
#include "H5Cpp.h"

namespace brics_3d {
namespace rsg {


/**
 * @brief Helper class to convert from and to HDF5 file format.
 *
 * @ingroup sceneGraph
 */
class HDF5Typecaster {
public:

	static const int rsgIdRank = 1;

	/* Constants and enums to serve as common agreement in the HDF5 encoding to
	 * form kind of a "protocol".
	 * Initialization of string constants: cf. below this file.
	 */
	static const string rsgIdName;
	static const string rsgNodeTypeInfoName;

	/*
	 * Top level indication on what to do with the data.
	 * Most cases will use ADD to perform an update on the
	 * robot scene graph.
	 */
	enum RsgUpdateCommand {
		UNKNOWN_COMMAND,
	    NONE,
	    ADD,
	    DELETE,
	    SET_ATTRIBUTES,
	    SET_TRANSFORM,
	    ADD_PARENT,
	    REMOVE_PARENT
	};

	/*
	 * In HDH5 there is only the "group". As we have in the robot scene
	 * graph further types like GeometricNode, Transform, etc. we need
	 * to indicate that with an enum.
	 */
	enum RsgNodeTypeInfo {
		UNKNOWN_NODE,
	    NODE,
	    GROUP,
	    GEOMETIRIC_NODE,
	    TRANSFORM,
	    UNCERTAIN_TRANSFORM,
	    JOINT
	};

	/*
	 * Enum to describe the robot scene graph geometry (= shape) data type.
	 */
	enum RsgShapeTypeInfo {
		UNKNOWN_SHAPE,
	    SHERE,
	    CYLINDER,
	    BOX,
	    POINT_CLOUD,
	    MESH
	};

	HDF5Typecaster(){};
	virtual ~HDF5Typecaster(){};

	inline static void addNodeIdToHDF5Group(brics_3d::rsg::Id id,  H5::Group& group) {
		hsize_t rsgIdDimensions[rsgIdRank];
		rsgIdDimensions[0] = 16; // 16 bytes
		H5::DataSpace rsgIdDataSpace(rsgIdRank, rsgIdDimensions);
		H5::IntType rsgIdDataType(H5::PredType::NATIVE_UCHAR); //for byte

		H5::DataSet rsgIdDataset = group.createDataSet(rsgIdName, rsgIdDataType, rsgIdDataSpace);
		rsgIdDataset.write(id.begin(), H5::PredType::NATIVE_UCHAR);
	}

	inline static void getNodeIdFromHDF5Group(brics_3d::rsg::Id& id, H5::Group& group) {
		H5::DataSet rsgIdDataset = group.openDataSet(rsgIdName);
		// Here might be some checks if dimension, etc match...
		rsgIdDataset.read(id.begin(), H5::PredType::NATIVE_UCHAR);
	}


	inline static void addAttributeToHDF5Group(brics_3d::rsg::Attribute attribute, H5::Group& group) {
		H5::StrType stringType(0, H5T_VARIABLE);
		H5::DataSpace attributeSpace(H5S_SCALAR);
		H5::Attribute rsgHDF5Attribute = group.createAttribute(attribute.key, stringType, attributeSpace);
		rsgHDF5Attribute.write(stringType, attribute.value);
	}

	inline static void addNodeTypeInfoToHDF5Group(RsgNodeTypeInfo nodeType, H5::Group& group) {
		H5::IntType rsgNodeTypeInfoDataType( H5::PredType::NATIVE_INT);
		H5::DataSpace rsgNodeTypeInfoSpace(H5S_SCALAR);
		H5::DataSet rsgNodeTypeInfoDataset = group.createDataSet("NodeTypeInfo" , rsgNodeTypeInfoDataType, rsgNodeTypeInfoSpace);
		rsgNodeTypeInfoDataset.write(&nodeType, rsgNodeTypeInfoDataType);
	}

	inline static void getNodeTypeInfoFromHDF5Group(RsgNodeTypeInfo& nodeType, H5::Group& group) {
		H5::IntType rsgNodeTypeInfoDataType( H5::PredType::NATIVE_INT);
		H5::DataSet rsgNodeTypeInfoDataset = group.openDataSet("NodeTypeInfo");
		rsgNodeTypeInfoDataset.read(&nodeType, rsgNodeTypeInfoDataType);
	}

};

/* initialization of all constants */
const string HDF5Typecaster::rsgIdName  = "Id";
const string HDF5Typecaster::rsgNodeTypeInfoName = "NodeTypeInfo";

} /* namespace rsg */
} /* namespace brics_3d */



#endif /* HDF5TYPECASTER_H_ */

/* EOF */
