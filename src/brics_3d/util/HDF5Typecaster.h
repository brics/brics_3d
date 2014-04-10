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
	static const int nonIndexedShapeRank = 1; // common rank for Sphere, Cylinder, Box
	static const int indexedShapeRank = 2; 	// common rank for PointCloud and Mesh data
	static const int transformDataRank = 1;	// array of structs
	static const int sphereDimension = 1; 		// radius
	static const int cylinderDimension = 2; 	// radius, height
	static const int boxDimension = 3; // x,y,z
	static const int matrixElements = 16;

	/* Constants and enums to serve as common agreement in the HDF5 encoding to
	 * form kind of a "protocol".
	 * Initialization of string constants: cf. below this file.
	 */
	static const string rsgIdName;
	static const string rsgNodeTypeInfoName;
	static const string rsgShapeName;
	static const string rsgShapeTypeInfoName;
	static const string rsgTransformName;

//	static const H5::PredType atomicRsgShapeType; // common atomic type for all shape data sets.

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
	    GEOMETIRC_NODE,
	    TRANSFORM,
	    UNCERTAIN_TRANSFORM,
	    JOINT
	};

	/*
	 * Enum to describe the robot scene graph geometry (= shape) data type.
	 */
	enum RsgShapeTypeInfo {
		UNKNOWN_SHAPE,
	    SPHERE,
	    CYLINDER,
	    BOX,
	    POINT_CLOUD,
	    MESH
	};

	 /* First structure and dataset*/
	typedef struct transform_data_t {
		double timeStamp;
		double matrixData[matrixElements];
	} transform_data_t;

	HDF5Typecaster(){};
	virtual ~HDF5Typecaster(){};

	inline static bool addNodeIdToHDF5Group(brics_3d::rsg::Id id,  H5::Group& group) {
		hsize_t rsgIdDimensions[rsgIdRank];
		rsgIdDimensions[0] = 16; // 16 bytes
		H5::DataSpace rsgIdDataSpace(rsgIdRank, rsgIdDimensions);
		H5::IntType rsgIdDataType(H5::PredType::NATIVE_UCHAR); //for byte

		H5::DataSet rsgIdDataset = group.createDataSet(rsgIdName, rsgIdDataType, rsgIdDataSpace);
		rsgIdDataset.write(id.begin(), H5::PredType::NATIVE_UCHAR);

		return true;
	}

	inline static bool getNodeIdFromHDF5Group(brics_3d::rsg::Id& id, H5::Group& group) {
		H5::DataSet rsgIdDataset = group.openDataSet(rsgIdName);
		// Here might be some checks if dimension, etc match...
		rsgIdDataset.read(id.begin(), H5::PredType::NATIVE_UCHAR);

		return true;
	}


	inline static bool addAttributeToHDF5Group(brics_3d::rsg::Attribute attribute, H5::Group& group) {
		H5::StrType stringType(0, H5T_VARIABLE);
		H5::DataSpace attributeSpace(H5S_SCALAR);
		H5::Attribute rsgHDF5Attribute = group.createAttribute(attribute.key, stringType, attributeSpace);
		rsgHDF5Attribute.write(stringType, attribute.value);

		return true;
	}

	inline static bool getAttributeByNameFromHDF5Group(brics_3d::rsg::Attribute& attribute, std::string name, H5::Group& group) {
		H5::StrType stringType(0, H5T_VARIABLE);

		try {
			H5::Attribute tmpAttribute = group.openAttribute(name);
			H5std_string valueString;
			tmpAttribute.read(stringType, valueString);
			attribute.key = name;
			attribute.value = valueString;

		} catch (H5::Exception e) {
			LOG(WARNING) << "Cannot get retrieve Attribute from HDF5 Group.";
			return false;
		}

		return true;
	}

	inline static bool addNodeTypeInfoToHDF5Group(RsgNodeTypeInfo nodeType, H5::Group& group) {
		H5::IntType rsgNodeTypeInfoDataType( H5::PredType::NATIVE_INT);
		H5::DataSpace rsgNodeTypeInfoSpace(H5S_SCALAR);
		H5::DataSet rsgNodeTypeInfoDataset = group.createDataSet(rsgNodeTypeInfoName , rsgNodeTypeInfoDataType, rsgNodeTypeInfoSpace);
		rsgNodeTypeInfoDataset.write(&nodeType, rsgNodeTypeInfoDataType);

		return true;
	}

	inline static bool getNodeTypeInfoFromHDF5Group(RsgNodeTypeInfo& nodeType, H5::Group& group) {
		H5::IntType rsgNodeTypeInfoDataType( H5::PredType::NATIVE_INT);
		H5::DataSet rsgNodeTypeInfoDataset = group.openDataSet(rsgNodeTypeInfoName);
		rsgNodeTypeInfoDataset.read(&nodeType, rsgNodeTypeInfoDataType);

		return true;
	}

	inline static bool addShapeToHDF5Group(brics_3d::rsg::Shape::ShapePtr shape, H5::Group& group) {
		LOG(DEBUG) << "convertShapeToHDF5DataSet: ";
		RsgShapeTypeInfo type;
		H5::DataSet rsgShapeDataset;
		H5::PredType atomicRsgShapeType = H5::PredType::NATIVE_DOUBLE; // common atomic type for all shape data sets.
		H5::FloatType rsgShapeDataType(atomicRsgShapeType);

		rsg::Sphere::SpherePtr sphere(new rsg::Sphere());
		sphere =  boost::dynamic_pointer_cast<rsg::Sphere>(shape);
		rsg::Box::BoxPtr box(new rsg::Box());
		box =  boost::dynamic_pointer_cast<rsg::Box>(shape);
		rsg::Cylinder::CylinderPtr cylinder(new rsg::Cylinder());
		cylinder =  boost::dynamic_pointer_cast<rsg::Cylinder>(shape);

		if (sphere !=0) {
			LOG(DEBUG) << "                 -> Found a sphere.";
			type = SPHERE;

			/* HDF5 meta data*/
			hsize_t rsgSphereDimension[nonIndexedShapeRank];
			rsgSphereDimension[0] = sphereDimension;
			H5::DataSpace rsgSphereDataSpace(nonIndexedShapeRank, rsgSphereDimension);

			/* actual data */
			double tmpSpehereData[sphereDimension];
			tmpSpehereData[0] = sphere->getRadius();
			rsgShapeDataset = group.createDataSet(rsgShapeName, rsgShapeDataType, rsgSphereDataSpace);
			rsgShapeDataset.write(&tmpSpehereData, atomicRsgShapeType);


		} else if (cylinder !=0) {
			LOG(DEBUG) << "                 -> Found a cylinder.";
			type = CYLINDER;

			/* HDF5 meta data*/
			hsize_t rsgCylinderDimension[nonIndexedShapeRank];
			rsgCylinderDimension[0] = cylinderDimension;
			H5::DataSpace rsgSphereDataSpace(nonIndexedShapeRank, rsgCylinderDimension);

			/* actual data */
			double tmpCylinderData[cylinderDimension];
			tmpCylinderData[0] = cylinder->getRadius();
			tmpCylinderData[1] = cylinder->getHeight();
			rsgShapeDataset = group.createDataSet(rsgShapeName, rsgShapeDataType, rsgSphereDataSpace);
			rsgShapeDataset.write(&tmpCylinderData, atomicRsgShapeType);

		} else if (box !=0) {
			LOG(DEBUG) << "                 -> Found a box.";
			type = BOX;

			/* HDF5 meta data*/
			hsize_t rsgBoxDimension[nonIndexedShapeRank];
			rsgBoxDimension[0] = boxDimension;
			H5::DataSpace rsgBoxDataSpace(nonIndexedShapeRank, rsgBoxDimension);

			/* actual data */
			double tmpBoxData[boxDimension];
			tmpBoxData[0] = box->getSizeX();
			tmpBoxData[1] = box->getSizeY();
			tmpBoxData[2] = box->getSizeZ();
			rsgShapeDataset = group.createDataSet(rsgShapeName, rsgShapeDataType, rsgBoxDataSpace);
			rsgShapeDataset.write(&tmpBoxData, atomicRsgShapeType);

		} else {
			LOG(ERROR) << "convertShapeToHDF5DataSet: Shape type not yet supported.";
			type = UNKNOWN_SHAPE;
			return false;
		}


		/* Attach the tag for the use type */
		H5::IntType rsgShapeTypeInfoDataType( H5::PredType::NATIVE_INT);
		H5::DataSpace rsgShapeTypeInfoSpace(H5S_SCALAR);
		H5::Attribute rsgHDF5Attribute = rsgShapeDataset.createAttribute(rsgShapeTypeInfoName, rsgShapeTypeInfoDataType, rsgShapeTypeInfoSpace);
		rsgHDF5Attribute.write(rsgShapeTypeInfoDataType, &type);

		/* Unit of Measurement */

		return true;
	}

	inline static bool getShapeFromHDF5Group(brics_3d::rsg::Shape::ShapePtr& shape, H5::Group& group) {
		LOG(DEBUG) << "getShapeFromHDF5Group: ";
		RsgShapeTypeInfo type;
		H5::PredType atomicRsgShapeType = H5::PredType::NATIVE_DOUBLE; // common atomic type for all shape data sets.
		H5::DataSet rsgShapeDataset;

		try {
			rsgShapeDataset = group.openDataSet(rsgShapeName);
		} catch (H5::Exception e) {
			LOG(ERROR) << "Cannot get open rsgShapeDataset from HDF5 Group.";
			return false;
		}

		/* Get tag for the used type */
		H5::IntType rsgShapeTypeInfoDataType( H5::PredType::NATIVE_INT);
		try {
			H5::Attribute tmpAttribute = rsgShapeDataset.openAttribute(rsgShapeTypeInfoName);
			tmpAttribute.read(rsgShapeTypeInfoDataType, &type);
			LOG(DEBUG) << "RsgShapeTypeInfo = " << type;
		} catch (H5::Exception e) {
			LOG(ERROR) << "Cannot get retrieve rsgShapeTypeInfoName from rsgShapeDataset.";
			return false;
		}

		brics_3d::rsg::Sphere::SpherePtr newSphere;
		brics_3d::rsg::Cylinder::CylinderPtr newCylinder;
		brics_3d::rsg::Box::BoxPtr newBox;

		switch (type) {
			case SPHERE:
				LOG(DEBUG) << "                 -> Found a new sphere.";

				double tmpShpere[sphereDimension];
				rsgShapeDataset.read(&tmpShpere, atomicRsgShapeType);
				// Here might be some checks if dimension, etc match...
				newSphere = brics_3d::rsg::Sphere::SpherePtr(new brics_3d::rsg::Sphere());
				newSphere->setRadius(tmpShpere[0]);
				shape = newSphere;

				break;

			case CYLINDER:
				LOG(DEBUG) << "                 -> Found a new cylinder.";

				double tmpCylinder[cylinderDimension];
				rsgShapeDataset.read(&tmpCylinder, atomicRsgShapeType);
				newCylinder = brics_3d::rsg::Cylinder::CylinderPtr(new brics_3d::rsg::Cylinder());
				newCylinder->setRadius(tmpCylinder[0]);
				newCylinder->setHeight(tmpCylinder[1]);
				shape = newCylinder;

				break;

			case BOX:
				LOG(DEBUG) << "                 -> Found a new box.";

				double tmpBox[boxDimension];
				rsgShapeDataset.read(&tmpBox, atomicRsgShapeType);
				newBox = brics_3d::rsg::Box::BoxPtr(new brics_3d::rsg::Box());
				newBox->setSizeX(tmpBox[0]);
				newBox->setSizeY(tmpBox[1]);
				newBox->setSizeZ(tmpBox[2]);
				shape = newBox;

				break;

			default:
				LOG(WARNING) << "                 -> Unknown or unsupported type " << type;
				break;
		}

		return true;
	}

	inline static bool addTransformToHDF5Group(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, H5::Group& group) {
		int numberOfTransforms = 1;

		const H5std_string MEMBER1( "timeStamp" );
		const H5std_string MEMBER2( "matrixData" );

		/* "type" for matrix array*/
		hsize_t rsgMatrixDataDimensions[rsgIdRank];
		rsgMatrixDataDimensions[0] = matrixElements; // 16
		H5::ArrayType rsgMatrixDataType(H5::PredType::NATIVE_DOUBLE, numberOfTransforms, rsgMatrixDataDimensions);

		/* assemble compound type */
		hsize_t rsgTransformDimensions[transformDataRank];
		rsgTransformDimensions[0] = numberOfTransforms; // number of transforms goes here
		H5::DataSpace rsgTransformDataSpace(transformDataRank, rsgTransformDimensions);
		H5::CompType rsgTransformDataType(sizeof(transform_data_t));
		rsgTransformDataType.insertMember(MEMBER1, HOFFSET(transform_data_t, timeStamp), H5::PredType::NATIVE_DOUBLE);
		rsgTransformDataType.insertMember(MEMBER2, HOFFSET(transform_data_t, matrixData), rsgMatrixDataType);

		/* prepare data */
		transform_data_t transforms[numberOfTransforms];
		transforms[0].timeStamp = timeStamp.getSeconds(); // for now just one element
		memcpy(&transforms[0].matrixData, transform->getRawData(), sizeof(double)*matrixElements);

		/* add data to HDF5 group */
		try {
			H5::DataSet rsgTransformDataset = group.createDataSet(rsgTransformName, rsgTransformDataType, rsgTransformDataSpace);
			rsgTransformDataset.write(&transforms, rsgTransformDataType);
		} catch (H5::Exception e) {
			LOG(ERROR) << "Cannot add transform data set to HDF5 group.";
			return false;
		}

		return true;
	}

	inline static bool getTransformFromHDF5Group(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform, TimeStamp& timeStamp, H5::Group& group) {
		int numberOfTransforms = 1;

		const H5std_string MEMBER1( "timeStamp" );
		const H5std_string MEMBER2( "matrixData" );

		/* "type" for matrix array*/
		hsize_t rsgMatrixDataDimensions[rsgIdRank];
		rsgMatrixDataDimensions[0] = matrixElements; // 16
		H5::ArrayType rsgMatrixDataType(H5::PredType::NATIVE_DOUBLE, numberOfTransforms, rsgMatrixDataDimensions);

		/* assemble compound type */
		hsize_t rsgTransformDimensions[transformDataRank];
		rsgTransformDimensions[0] = numberOfTransforms; // number of transforms goes here
		H5::DataSpace rsgTransformDataSpace(transformDataRank, rsgTransformDimensions);
		H5::CompType rsgTransformDataType(sizeof(transform_data_t));
		rsgTransformDataType.insertMember(MEMBER1, HOFFSET(transform_data_t, timeStamp), H5::PredType::NATIVE_DOUBLE);
		rsgTransformDataType.insertMember(MEMBER2, HOFFSET(transform_data_t, matrixData), rsgMatrixDataType);

		/* do read from group */
		try {
			H5::DataSet rsgTransformDataset = group.openDataSet(rsgTransformName);
//			H5::DataSpace rsgReadTransformDataSpace = rsgTransformDataset.getSpace();
//			rsgReadTransformDataSpace.getCounter();
			transform_data_t transforms[numberOfTransforms];
			rsgTransformDataset.read(&transforms, rsgTransformDataType);

			/* process data */
			timeStamp = brics_3d::rsg::TimeStamp(transforms[0].timeStamp, brics_3d::Units::Second);
			memcpy(transform->setRawData(), &transforms[0].matrixData, sizeof(double)*matrixElements);
		} catch (H5::Exception e) {
			LOG(ERROR) << "Cannot retrieve transform data from HDF5 group.";
			return false;
		}

		return true;
	}

};

/* initialization of all constants */
const string HDF5Typecaster::rsgIdName  = "Id";
const string HDF5Typecaster::rsgNodeTypeInfoName = "NodeTypeInfo";
const string HDF5Typecaster::rsgShapeName = "Shape";
const string HDF5Typecaster::rsgShapeTypeInfoName = "ShapeTypeInfo";
const string HDF5Typecaster::rsgTransformName = "Transform";

//const H5::PredType atomicRsgShapeType = H5::PredType::NATIVE_DOUBLE;

} /* namespace rsg */
} /* namespace brics_3d */



#endif /* HDF5TYPECASTER_H_ */

/* EOF */
