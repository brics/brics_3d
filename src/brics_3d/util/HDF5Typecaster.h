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

#ifndef RSG_HDF5TYPECASTER_H_
#define RSG_HDF5TYPECASTER_H_

#ifdef HDF_1_8_12_OR_HIGHER
#include <H5Cpp.h>
#else
#include <cpp/H5Cpp.h>
#endif

#include "brics_3d/core/Logger.h"
#include "brics_3d/worldModel/sceneGraph/Attribute.h"
#include "brics_3d/worldModel/sceneGraph/Shape.h"
#include "brics_3d/worldModel/sceneGraph/Sphere.h"
#include "brics_3d/worldModel/sceneGraph/Box.h"
#include "brics_3d/worldModel/sceneGraph/Cylinder.h"
#include "brics_3d/worldModel/sceneGraph/PointCloud.h"
#include "brics_3d/worldModel/sceneGraph/Mesh.h"
#include "brics_3d/core/ColoredPoint3D.h"

/* Constants to serve as common agreement in the HDF5 encoding to
 * form kind of a "protocol".
 */
#define rsgCommandTypeInfoName "CommandTypeInfo"
#define rsgIdName "Id"
#define rsgParentIdName "ParentId"
#define rsgNodeTypeInfoName "NodeTypeInfo"
#define rsgShapeName "Shape"
#define rsgShapeTypeInfoName "ShapeTypeInfo"
#define rsgPointCloudTypeInfoName "PointCloudTypeInfo"
#define rsgTransformName "Transform"
#define rsgTimeStampName "TimeStamp"

//#define HDF_1_8_12_OR_HIGHER

namespace brics_3d {
namespace rsg {

#ifdef HDF_1_8_12_OR_HIGHER
static void collectHDF5AttributeNames(H5::H5Location& loc,
#else
static void collectHDF5AttributeNames(H5::H5Object& loc,
#endif
		const H5std_string attr_name, void *operator_data) {

	LOG(DEBUG) << "H5::Attribute name = " << attr_name;
	vector<std::string>* attributeNames = (vector<std::string>*)operator_data;
	attributeNames->push_back(attr_name);
}

/**
 * @brief Helper class to convert from and to HDF5 file format.
 *
 * This class is designed as "header only". It can be always included in other
 * projects, regardless the used compilation flags (USE_HDF5) of brics_3d.
 *
 * @ingroup sceneGraph
 */
class HDF5Typecaster {
public:

	static const int rsgIdRank = 1;
	static const int nonIndexedShapeRank = 1; // common rank for Sphere, Cylinder, Box
	static const int indexedShapeRank = 2; 	// common rank for PointCloud and Mesh data
	static const int transformDataRank = 1;	// array of structs
	static const int pointCloudDataRank = 1;	// array of structs
	static const int sphereDimension = 1; 		// radius
	static const int cylinderDimension = 2; 	// radius, height
	static const int boxDimension = 3; // x,y,z
	static const int matrixElements = 16;
	static const int rsgTimeStampRank = 1;

	/* Constants and enums to serve as common agreement in the HDF5 encoding to
	 * form kind of a "protocol".
	 * Initialization of string constants: cf. below this file.
	 */
//	static const std::string rsgIdName;
//	static const std::string rsgNodeTypeInfoName;
//	static const std::string rsgShapeName;
//	static const std::string rsgShapeTypeInfoName;
//	static const std::string rsgTransformName;

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
	    REMOVE_PARENT,
	    ADD_REMOTE_ROOT_NODE
	};

	/*
	 * In HDF5 there is only the "group". As we have in the robot scene
	 * graph further types like GeometricNode, Transform, etc. we need
	 * to indicate that with an enum.
	 */
	enum RsgNodeTypeInfo {
		UNKNOWN_NODE,
	    NODE,
	    GROUP,
	    CONNECTION,
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

	/* Compound data type for (temporal cached) transform data */
	typedef struct transform_data_t {
		double timeStamp;
		double matrixData[matrixElements];
	} transform_data_t;

	/* Compound data type for XYZ point cloud */
	typedef struct point_cloud_xyz_data_t {
		double x;
		double y;
		double z;
	} point_cloud_xyz_data_t;

	/* Compound data type for XYZRGB point cloud */
	typedef struct point_cloud_xyzrgb_data_t {
		double x;
		double y;
		double z;
		unsigned char r;
		unsigned char g;
		unsigned char b;
	} point_cloud_xyzrgb_data_t;

	HDF5Typecaster(){};
	virtual ~HDF5Typecaster(){};

	inline static bool addCommandTypeInfoToHDF5Group(RsgUpdateCommand commandType, H5::Group& group) {
		H5::IntType rsgCommandTypeInfoDataType( H5::PredType::NATIVE_INT);
		H5::DataSpace rsgCommandTypeInfoSpace(H5S_SCALAR);
		H5::DataSet rsgCommandTypeInfoDataset = group.createDataSet(rsgCommandTypeInfoName, rsgCommandTypeInfoDataType, rsgCommandTypeInfoSpace);
		rsgCommandTypeInfoDataset.write(&commandType, rsgCommandTypeInfoDataType);

		return true;
	}

	inline static bool getCommandTypeInfoFromHDF5Group(RsgUpdateCommand& commandType, H5::Group& group) {
		H5::IntType rsgCommandTypeInfoDataType( H5::PredType::NATIVE_INT);
		H5::DataSet rsgCommandTypeInfoDataset = group.openDataSet(rsgCommandTypeInfoName);
		rsgCommandTypeInfoDataset.read(&commandType, rsgCommandTypeInfoDataType);

		return true;
	}

	inline static bool addNodeIdToHDF5Group(brics_3d::rsg::Id id,  H5::Group& group, std::string idName = rsgIdName) {
		hsize_t rsgIdDimensions[rsgIdRank];
		rsgIdDimensions[0] = 16; // 16 bytes
		H5::DataSpace rsgIdDataSpace(rsgIdRank, rsgIdDimensions);
		H5::IntType rsgIdDataType(H5::PredType::NATIVE_UCHAR); //for byte

		H5::DataSet rsgIdDataset = group.createDataSet(idName, rsgIdDataType, rsgIdDataSpace);
		rsgIdDataset.write(id.begin(), H5::PredType::NATIVE_UCHAR);

		return true;
	}

	inline static bool getNodeIdFromHDF5Group(brics_3d::rsg::Id& id, H5::Group& group, std::string idName = rsgIdName) {
		H5::DataSet rsgIdDataset = group.openDataSet(idName);
		// Here might be some checks if dimension, etc match...
		rsgIdDataset.read(id.begin(), H5::PredType::NATIVE_UCHAR);

		return true;
	}

	inline static bool addIdsToHDF5Group(vector<brics_3d::rsg::Id> ids,  H5::Group& group, std::string idName) {
		hsize_t rsgIdDimensions[rsgIdRank];
		rsgIdDimensions[0] = 16 * ids.size(); // 16 * N bytes
		H5::DataSpace rsgIdDataSpace(rsgIdRank, rsgIdDimensions);
		H5::IntType rsgIdDataType(H5::PredType::NATIVE_UCHAR); //for byte

		H5::DataSet rsgIdsDataset = group.createDataSet(idName, rsgIdDataType, rsgIdDataSpace);
		// Write out as continious byte space
		for (vector<brics_3d::rsg::Id>::iterator it = ids.begin(); it!=ids.end(); ++it) {
			rsgIdsDataset.write(it->begin(), H5::PredType::NATIVE_UCHAR);
		}

		return true;
	}

	inline static bool getIdsFromHDF5Group(vector<brics_3d::rsg::Id>& ids, H5::Group& group, std::string idName) {
		H5::DataSet rsgIdsDataset = group.openDataSet(idName);


		/* Go through HDF5 meta-data */
		int numberOfIds = 0;
		H5::DataSpace rsgInferredIdDataSpace = rsgIdsDataset.getSpace();
		numberOfIds = rsgInferredIdDataSpace.getSimpleExtentNpoints();
		unsigned char rawData [16*numberOfIds];
//		rsgIdsDataset.read(rawData, H5::PredType::NATIVE_UCHAR);
//		for (int i = 0; i < numberOfIds; ++i) {
//			brics_3d::rsg::Id id;
//			rsgIdsDataset.read(id.begin(), H5::PredType::NATIVE_UCHAR);
//		}

		return false;
	}

	inline static bool addTimeStampToHDF5Group(brics_3d::rsg::TimeStamp stamp,  H5::Group& group, std::string timeStampName) {
		hsize_t rsgTimeStampDimensions[rsgTimeStampRank];
		rsgTimeStampDimensions[0] = 1; // 1 DOUBLE VALUE
		H5::DataSpace rsgTimeStampDataSpace(rsgTimeStampRank, rsgTimeStampDimensions);
		H5::FloatType rsgTimeStampDataType(H5::PredType::NATIVE_DOUBLE); //for byte

		H5::DataSet rsgTimeStampDataset = group.createDataSet(timeStampName, rsgTimeStampDataType, rsgTimeStampDataSpace);
		double tmpStamp = stamp.getSeconds();
		rsgTimeStampDataset.write(&tmpStamp, H5::PredType::NATIVE_DOUBLE);

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

	inline static bool addAttributesToHDF5Group(std::vector<brics_3d::rsg::Attribute> attributes, H5::Group& group) {
		for (std::vector<brics_3d::rsg::Attribute>::iterator it = attributes.begin(); it != attributes.end(); ++it) {
			if(!addAttributeToHDF5Group(*it, group)) {
				return false;
			}
		}

		return true;
	}

	inline static bool getAttributesFromHDF5Group(std::vector<brics_3d::rsg::Attribute>& attributes, H5::Group& group) {
		LOG(DEBUG) << "H5::Group has " << group.getNumAttrs() << " Attribute(s)";
		vector<std::string> attributeNames;
		group.iterateAttrs(collectHDF5AttributeNames, NULL, &attributeNames);

		for (vector<std::string>::iterator it = attributeNames.begin(); it!=attributeNames.end(); ++it) {
			brics_3d::rsg::Attribute result;
			if(brics_3d::rsg::HDF5Typecaster::getAttributeByNameFromHDF5Group(result, *it, group)) {
				LOG(DEBUG) << "\t adding rsg::Attribute " << result;
				attributes.push_back(result);
			}
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

		} else if (shape->getPointCloudIterator() != 0) {

			LOG(DEBUG) << "                 -> Found a point cloud.";
			type = POINT_CLOUD;

			/* HDF5 meta data: we treate a single row as a compund data type */
			const H5std_string MEMBER1( "x" );
			const H5std_string MEMBER2( "y" );
			const H5std_string MEMBER3( "z" );
			const H5std_string MEMBER4( "r" );
			const H5std_string MEMBER5( "g" );
			const H5std_string MEMBER6( "b" );

			/* as there is no zize method in the iterator we have to loop ofer it a priori */
			int numberOfPoints = 0;
			IPoint3DIterator::IPoint3DIteratorPtr it = shape->getPointCloudIterator();
			for (it->begin(); !it->end(); it->next()) {
				numberOfPoints++;
			}
			LOG(DEBUG) << "numberOfPoints = " << numberOfPoints;

			hsize_t rsgPointCloutDimensions[pointCloudDataRank]; // on dim (array)
			rsgPointCloutDimensions[0] = numberOfPoints; // number of nD point in point cloud gies here
			H5::DataSpace rsgPointDataSpace(pointCloudDataRank, rsgPointCloutDimensions);
			H5::CompType rsgPointCloudDataType(sizeof(point_cloud_xyzrgb_data_t));
			rsgPointCloudDataType.insertMember(MEMBER1, HOFFSET(point_cloud_xyzrgb_data_t, x), H5::PredType::NATIVE_DOUBLE);
			rsgPointCloudDataType.insertMember(MEMBER2, HOFFSET(point_cloud_xyzrgb_data_t, y), H5::PredType::NATIVE_DOUBLE);
			rsgPointCloudDataType.insertMember(MEMBER3, HOFFSET(point_cloud_xyzrgb_data_t, z), H5::PredType::NATIVE_DOUBLE);
			rsgPointCloudDataType.insertMember(MEMBER4, HOFFSET(point_cloud_xyzrgb_data_t, r), H5::PredType::NATIVE_UCHAR);
			rsgPointCloudDataType.insertMember(MEMBER5, HOFFSET(point_cloud_xyzrgb_data_t, g), H5::PredType::NATIVE_UCHAR);
			rsgPointCloudDataType.insertMember(MEMBER6, HOFFSET(point_cloud_xyzrgb_data_t, b), H5::PredType::NATIVE_UCHAR);


			/* Prepare actual data */
			point_cloud_xyzrgb_data_t points[numberOfPoints]; // raw version of point cloud
			int i = 0;
			for (it->begin(); !it->end(); it->next()) {
				points[i].x = it->getX();
				points[i].y = it->getY();
				points[i].z = it->getZ();


				if(it->getRawData()->asColoredPoint3D() != 0) {
					points[i].r = it->getRawData()->asColoredPoint3D()->getR();
					points[i].g = it->getRawData()->asColoredPoint3D()->getG();
					points[i].b = it->getRawData()->asColoredPoint3D()->getB();
				} else {
					points[i].r = 0x00;
					points[i].g = 0x00;
					points[i].b = 0x00;
				}

				i++;
				if (i >= numberOfPoints) { // early termination (just in case)
					break;
				}
			}

			/* Write out data */
			rsgShapeDataset = group.createDataSet(rsgShapeName, rsgPointCloudDataType, rsgPointDataSpace);
			rsgShapeDataset.write(&points, rsgPointCloudDataType);

			/* Attach an additional attribute indicating the underlying type of the point cloud (e.g. brics_3d::PointCloud3D) */
			H5::StrType stringType(0, H5T_VARIABLE);
			H5::DataSpace attributeSpace(H5S_SCALAR);
			H5::Attribute rsgHDF5Attribute = rsgShapeDataset.createAttribute(rsgPointCloudTypeInfoName, stringType, attributeSpace);
			rsgHDF5Attribute.write(stringType, it->getPointCloudTypeName());

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

		/* Additional matadata in case of a point cloud */
		H5std_string pointCloudName = "unspecifiedPointcloudType";
		H5::Attribute pointCloudAttribute;
		H5::StrType stringType(0, H5T_VARIABLE);

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

			case POINT_CLOUD:
				LOG(DEBUG) << "                 -> Found a new point cloud.";

				try {
					pointCloudAttribute = rsgShapeDataset.openAttribute(rsgPointCloudTypeInfoName);
					pointCloudAttribute.read(stringType, pointCloudName);
					LOG(DEBUG) << "                 rsgPointCloudTypeInfoName = " << pointCloudName;
				} catch (H5::Exception e) {
					LOG(ERROR) << "Cannot get retrieve rsgPointCloudTypeInfoName from rsgShapeDataset.";
					return false;
				}

				/* PointCloudFactory? */
				if(pointCloudName.compare("brics_3d::PointCloud3D") == 0) {
					LOG(DEBUG) << "                 BRICS_3D point cloud detected.";

					/* Create a new (concrete) point cloud  */
					brics_3d::PointCloud3D::PointCloud3DPtr newPointCloud(new brics_3d::PointCloud3D());

					/* Create scene graph container for specific point cloud type  */
					brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>::PointCloudPtr newPointCloudContainer(new brics_3d::rsg::PointCloud<brics_3d::PointCloud3D>());

					/* Assign concrete type to container */
					newPointCloudContainer->data=newPointCloud;

					/* HDF5 meta data: we treate a single row as a compund data type */
					const H5std_string MEMBER1( "x" );
					const H5std_string MEMBER2( "y" );
					const H5std_string MEMBER3( "z" );
					const H5std_string MEMBER4( "r" );
					const H5std_string MEMBER5( "g" );
					const H5std_string MEMBER6( "b" );


					/* Go through HDF5 meta-data */
					int numberOfPoints = 0;
					H5::DataSpace rsgInferredPointDataSpace = rsgShapeDataset.getSpace();
					numberOfPoints = rsgInferredPointDataSpace.getSimpleExtentNpoints();

					H5::CompType rsgPointCloudDataType(sizeof(point_cloud_xyzrgb_data_t));
					rsgPointCloudDataType.insertMember(MEMBER1, HOFFSET(point_cloud_xyzrgb_data_t, x), H5::PredType::NATIVE_DOUBLE);
					rsgPointCloudDataType.insertMember(MEMBER2, HOFFSET(point_cloud_xyzrgb_data_t, y), H5::PredType::NATIVE_DOUBLE);
					rsgPointCloudDataType.insertMember(MEMBER3, HOFFSET(point_cloud_xyzrgb_data_t, z), H5::PredType::NATIVE_DOUBLE);
					rsgPointCloudDataType.insertMember(MEMBER4, HOFFSET(point_cloud_xyzrgb_data_t, r), H5::PredType::NATIVE_UCHAR);
					rsgPointCloudDataType.insertMember(MEMBER5, HOFFSET(point_cloud_xyzrgb_data_t, g), H5::PredType::NATIVE_UCHAR);
					rsgPointCloudDataType.insertMember(MEMBER6, HOFFSET(point_cloud_xyzrgb_data_t, b), H5::PredType::NATIVE_UCHAR);

					/* Intermediate buffer for point cloud (TODO: remove this step) */
					point_cloud_xyzrgb_data_t points[numberOfPoints]; // raw version of point cloud

					/* Fill point cloud  with data */
					LOG(DEBUG) << "                 Update contains " << numberOfPoints << " points.";
					rsgShapeDataset.read(&points, rsgPointCloudDataType);

					for (int i = 0; i < numberOfPoints; ++i) {

						Point3D* tmpPoint =  new Point3D(
								points[i].x,
								points[i].y,
								points[i].z);

						ColoredPoint3D* tmpColoredPoint = new ColoredPoint3D(tmpPoint, //optional decoration layer
								points[i].r,
								points[i].g,
								points[i].b);

						newPointCloudContainer->data->addPointPtr(tmpColoredPoint);
					}

					shape = newPointCloudContainer;

				} else {
					LOG(WARNING) << "                 " << pointCloudName << " - this point cloud type cannot be deserialized.";
					return false;
				}

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

	inline static bool addTimeStampToHDF5Group(brics_3d::rsg::TimeStamp timeStamp, H5::Group& group) {
		H5::IntType rsgTimeStampType( H5::PredType::NATIVE_DOUBLE);
		H5::DataSpace rsgTimeStampSpace(H5S_SCALAR);
		H5::DataSet rsgTimeStampDataset = group.createDataSet(rsgTimeStampName, rsgTimeStampType, rsgTimeStampSpace);
		double tmpTimeStamp = timeStamp.getSeconds();
		rsgTimeStampDataset.write(&tmpTimeStamp, rsgTimeStampType);

		return true;
	}

	inline static bool getTimeStampFromHDF5Group(brics_3d::rsg::TimeStamp& timeStamp, H5::Group& group) {
		H5::IntType rsgTimeStampType( H5::PredType::NATIVE_DOUBLE);
		H5::DataSet rsgTimeStampDataset = group.openDataSet(rsgTimeStampName);
		double tmpTimeStamp;
		rsgTimeStampDataset.read(&tmpTimeStamp, rsgTimeStampType);
		timeStamp = brics_3d::rsg::TimeStamp(tmpTimeStamp, brics_3d::Units::Second);

		return true;
	}

};

/* initialization of all constants */
//const string HDF5Typecaster::rsgIdName  = "Id";
//const string HDF5Typecaster::rsgNodeTypeInfoName = "NodeTypeInfo";
//const string HDF5Typecaster::rsgShapeName = "Shape";
//const string HDF5Typecaster::rsgShapeTypeInfoName = "ShapeTypeInfo";
//const string HDF5Typecaster::rsgTransformName = "Transform";

//const H5::PredType atomicRsgShapeType = H5::PredType::NATIVE_DOUBLE;

} /* namespace rsg */
} /* namespace brics_3d */



#endif /* RSG_HDF5TYPECASTER_H_ */

/* EOF */
