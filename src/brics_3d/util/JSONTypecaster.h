/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2015, KU Leuven
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

#ifndef RSG_JSONTYPECASTER_H_
#define RSG_JSONTYPECASTER_H_

#include "brics_3d/core/Logger.h"
#include "brics_3d/core/HomogeneousMatrix44.h"
#include "brics_3d/core/ColoredPoint3D.h"
#include "brics_3d/core/TriangleMeshImplicit.h"
#include "brics_3d/worldModel/WorldModel.h"
#include <Variant/Variant.h>
#include <Variant/SchemaLoader.h>

namespace brics_3d {
namespace rsg {

/**
 * @brief Helper class to convert from and to JSON file format.
 *
 * This class is designed as "header only". It can be always included in other
 * projects, regardless the used compilation flags (USE_JSON) of brics_3d.
 *
 * @ingroup sceneGraph
 */
class JSONTypecaster {
public:

	inline static bool stringToJSON(string modelAsString, libvariant::Variant& model) {
		try {
			model = libvariant:: Deserialize(modelAsString, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON
		} catch (std::exception const & e) {
			LOG(ERROR) << "JSONTypecaster::stringToJSON: Parser error: " << e.what() << std::endl;
			return false;
		}
		return true;
	}

	inline static bool JSONtoString(libvariant::Variant model, string& modelAsString) {
		try {
			modelAsString = libvariant::Serialize(model, libvariant::SERIALIZE_JSON);
		} catch (std::exception const & e) {
			LOG(ERROR) << "JSONTypecaster::JSONtoSting: Parser error: " << e.what() << std::endl;
			return false;
		}
		return true;
	}

	inline static bool validateFunctionBlockModel (libvariant::Variant model, string metaModel, string path, string& errorModel){
		libvariant::Variant schema = libvariant::Variant(metaModel);
		libvariant::AdvSchemaLoader loader;
		loader.AddPath(path);
		libvariant::SchemaResult result = libvariant::SchemaValidate(schema, model, &loader);
		if (result.Error()) {
			LOG(DEBUG) << "JSON Validator: Model validation failed: " << result;
			std::stringstream tmpErrorModel("");
			tmpErrorModel << "{\"error\": {\"message\": \" " << result.PrettyPrintMessage() <<"\"}}";
			errorModel =  tmpErrorModel.str();
			return false;
		}
		return true;
	}

	inline static rsg::Id getIdFromJSON(libvariant::Variant& node, string idTag) {
		rsg::Id id = 0; //NiL

		if(node.Contains(idTag)) { // required
			if(!id.fromString(node.Get(idTag).AsString())) {
				LOG(ERROR) << "Can not parse model. Invalid value = " << node.Get(idTag).AsString() << " for node Id " << idTag << " specified.";
			}
			LOG(DEBUG) << "JSONTypecaster: Id " << idTag << " is = " << id;
		} else {
			LOG(WARNING) << "Can not parse model. No node Id " << idTag << " specified.";
		}

		return id;
	}

	inline static bool addIdToJSON(brics_3d::rsg::Id id, libvariant::Variant& node, string idTag) {

		node.Set(idTag, libvariant::Variant(id.toString()));

		return true;
	}

	inline static vector<rsg::Id> getIdsFromJSON(libvariant::Variant& node, string idsTag) {
		vector<rsg::Id> ids;

		try {
			LOG(DEBUG) << "JSONTypecaster: Id list " << idsTag << " has the following Ids:";
			libvariant::Variant idList = node.Get(idsTag);
			if (idList.IsList()) {
				for (libvariant::Variant::ListIterator i(idList.ListBegin()), e(idList.ListEnd()); i!=e; ++i) {
					Id id;
					if(!id.fromString(i->AsString())) {
						LOG(ERROR) << "Can not parse model. Invalid value = " << i->AsString() << " for node Id in " << idsTag << " specified.";
					} else {
						LOG(DEBUG) << "JSONTypecaster: \t " << id;
						ids.push_back(id);
					}
				}
			}

		} catch (std::exception const & e) {
			//simply return am empty list
			ids.clear();
		}
		return ids;
	}

	inline static bool addIdsToJSON(vector<rsg::Id>& ids, libvariant::Variant& node, string idsTag) {
		libvariant::Variant idsAsJson(libvariant::VariantDefines::ListType);

		for (std::vector<brics_3d::rsg::Id>::iterator it = ids.begin(); it != ids.end(); ++it) {
			idsAsJson.Append(libvariant::Variant(it->toString()));
		}

		node.Set(idsTag, idsAsJson);

		return true;
	}

	inline static rsg::TimeStamp getTimeStampFromJSON(libvariant::Variant& node, string stampTag) {
		rsg::TimeStamp stamp(-1.0); //-inf?

		if(node.Contains(stampTag)) {
			LOG(DEBUG) << "JSONTypecaster: entity has the following time stamp:";
			libvariant::Variant stampModel = node.Get(stampTag);

			if(stampModel.Contains("@stamptype")) {
				if(!stampModel.Contains("stamp")) {
					LOG(ERROR) << "JSONTypecaster: Time stamp entity has no @stamptype tag.";
					return stamp;
				}

				if( stampModel.Get("@stamptype").AsString().compare("TimeStampUTCms") == 0 ) {

					stamp = TimeStamp( stampModel.Get("stamp").AsDouble(), Units::MilliSecond);

				} else if ( stampModel.Get("@stamptype").AsString().compare("TimeStampDate") == 0 ) {

					string dateTime =  stampModel.Get("stamp").AsString();
					struct tm tmlol;
					strptime(dateTime.c_str(), "%Y-%m-%dT%H:%M:%SZ", &tmlol);
					time_t t = mktime(&tmlol); // Seconds since epoche
					stamp = TimeStamp(t, Units::Second);

				} else {
					LOG(ERROR) << "JSONTypecaster: Time stamp definition has unknown type @stamptype identifier";
				}

			} else {
				LOG(ERROR) << "JSONTypecaster: Time stamp definition has no type @stamptype identifier";
			}
		} else {
			LOG(ERROR) << "JSONTypecaster: Time stamp entity has no stamp tag: " << stampTag;
		}

		LOG(DEBUG) << "JSONTypecaster: stamp = " << stamp.getSeconds();

		return stamp;
	}

	inline static bool addTimeStampToJSON(rsg::TimeStamp timeStamp, libvariant::Variant& node, string stampTag) {
		libvariant::Variant stampModel;

		stampModel.Set("@stamptype", libvariant::Variant("TimeStampUTCms"));
		stampModel.Set("stamp", timeStamp.getSeconds() * Units::getScale(Units::Second, Units::MilliSecond) );

		node.Set(stampTag, stampModel);

		return true;
	}

	inline static std::vector<rsg::Attribute> getAttributesFromJSON (libvariant::Variant& node, std::string attributesTag = "attributes") {
		std::vector<rsg::Attribute> attributes;
		attributes.clear();

		if(node.Contains(attributesTag)) {
			LOG(DEBUG) << "JSONTypecaster: Node has the following attributes:";

			libvariant::Variant attributeList = node.Get(attributesTag);
			if (attributeList.IsList()) {
				for (libvariant::Variant::ListIterator i(attributeList.ListBegin()), e(attributeList.ListEnd()); i!=e; ++i) {
					if(!i->Contains("key")) {
						LOG(ERROR) << "JSONTypecaster: Attribute has no key tag. Aborting.";
						attributes.clear(); // This is a rather strict abort.
						return attributes;
					}
					if(!i->Contains("value")) {
						LOG(ERROR) << "JSONTypecaster: Attribute has no value tag. Aborting.";
						attributes.clear(); // This is a rather strict abort.
						return attributes;
					}

					try {

						LOG(DEBUG) << "\t( " << i->Get("key").AsString() << ", " << i->Get("value").AsString() << " )";
						attributes.push_back(rsg::Attribute(i->Get("key").AsString(), i->Get("value").AsString()));

					/*
					 * Here we are rather permissive on what exactly a value is.
					 * We actually allow for a complete JSON model here.
					 */
					} catch (libvariant::UnableToConvertError e) {

						LOG(DEBUG) << "JSONTypecaster: parsing a non string value";
						string complexValue = libvariant::Serialize(i->Get("value"), libvariant::SERIALIZE_JSON);

						LOG(DEBUG) << "\t( " << i->Get("key").AsString() << ", " << complexValue << " )";
						attributes.push_back(rsg::Attribute(i->Get("key").AsString(), complexValue));
					}
				}
			}
		}

		return attributes;
	}

	inline static bool addAttributesToJSON(std::vector<brics_3d::rsg::Attribute> attributes, libvariant::Variant& node, std::string attributesTag = "attributes") {
		libvariant::Variant attributesAsJson(libvariant::VariantDefines::ListType);

		for (std::vector<brics_3d::rsg::Attribute>::iterator it = attributes.begin(); it != attributes.end(); ++it) {
			libvariant::Variant attributeAsJson;
			attributeAsJson.Set("key", libvariant::Variant(it->key));
			LOG(DEBUG) << "JSONTypecaster: adding Attributes to JSON: ( " << it->key << ", "<< it->value << " )";
			try {
				libvariant::Variant valueAsModel = libvariant::Deserialize(it->value, libvariant::SERIALIZE_GUESS); // GUESS seems to be more permissive with parsing than JSON
				attributeAsJson.Set("value", valueAsModel);
			} catch (std::exception e) {
				LOG(DEBUG) << "JSONTypecaster: using simple value format";
				attributeAsJson.Set("value", libvariant::Variant(it->value));
			}
//			attributeAsJson.Set("value", libvariant::Variant(it->value));
//			attributeAsJson.Set("value", valueAsModel);
			attributesAsJson.Append(attributeAsJson);
		}

		node.Set(attributesTag, attributesAsJson);

		return true;
	}

	inline static bool getTransformCacheFromJSON(TemporalCache<IHomogeneousMatrix44::IHomogeneousMatrix44Ptr>& history, libvariant::Variant& node) {
		history.clear();

		if(node.Contains("history")) {
					LOG(DEBUG) << "JSONTypecaster: Transform has the following history:";

					libvariant::Variant cacheList = node.Get("history");
					if (cacheList.IsList()) {
						for (libvariant::Variant::ListIterator i(cacheList.ListBegin()), e(cacheList.ListEnd()); i!=e; ++i) {
							TimeStamp stamp = getTimeStampFromJSON(*i, "stamp");
							if(stamp < TimeStamp(0.0)) {
								LOG(ERROR) << "JSONTypecaster: Transform history contains an invalid time stamp. Aborting.";
								// We keep the so far history, to allow parsing until first occurrence of corrupted data (in larger time series).
								return false;
							}

							if(!i->Contains("transform")) {
								LOG(ERROR) << "JSONTypecaster: Transform history has no transform tag. Aborting.";
								// We keep the so far history, to allow parsing until first occurrence of corrupted data (in larger time series).
								return false;
							}
							libvariant::Variant transformModel = i->Get("transform");

							if(!transformModel.Contains("type")) {
								LOG(ERROR) << "JSONTypecaster: Transform history has no transform type. Aborting.";
								// We keep the so far history, to allow parsing until first occurrence of corrupted data (in larger time series).
								return false;
							}


							if( transformModel.Get("type").AsString().compare("HomogeneousMatrix44") == 0 ) {
							     /*
							      * Coefficient layout for rotation part:
							      * r0 r1 r2
							      * r3 r4 r5
							      * r6 r7 r8
							      */
								HomogeneousMatrix44::IHomogeneousMatrix44Ptr transform(new HomogeneousMatrix44(
										// rotation
										transformModel.Get("matrix").At(0).At(0).AsDouble(),
										transformModel.Get("matrix").At(0).At(1).AsDouble(),
										transformModel.Get("matrix").At(0).At(2).AsDouble(),

										transformModel.Get("matrix").At(1).At(0).AsDouble(),
										transformModel.Get("matrix").At(1).At(1).AsDouble(),
										transformModel.Get("matrix").At(1).At(2).AsDouble(),

										transformModel.Get("matrix").At(2).At(0).AsDouble(),
										transformModel.Get("matrix").At(2).At(1).AsDouble(),
										transformModel.Get("matrix").At(2).At(2).AsDouble(),
										// translation
										transformModel.Get("matrix").At(0).At(3).AsDouble(),
										transformModel.Get("matrix").At(1).At(3).AsDouble(),
										transformModel.Get("matrix").At(2).At(3).AsDouble()
								));

								LOG(DEBUG) << "transform data = " << *transform;
								history.insertData(transform, stamp);

							} else {
								LOG(DEBUG) << "JSONTypecaster: Transform uses unknown type";
							}

						}
					}
				}


		return true;
	}

	inline static bool addTransformCacheToJSON(TemporalCache<IHomogeneousMatrix44::IHomogeneousMatrix44Ptr>& history, libvariant::Variant& node) {
		libvariant::Variant historyModel(libvariant::VariantDefines::ListType);


		for (std::vector<std::pair<IHomogeneousMatrix44::IHomogeneousMatrix44Ptr, TimeStamp> >::const_iterator it = history.begin(); it != history.end(); ++it) {
			libvariant::Variant transformEntryAsJson;

			/* time stamp */
			addTimeStampToJSON(it->second, transformEntryAsJson, "stamp");


			/* transform data */
			const double* matrixData = it->first->getRawData();
			libvariant::Variant transformModel;
			transformModel.Set("type", libvariant::Variant("HomogeneousMatrix44"));

			libvariant::Variant matrix(libvariant::VariantDefines::ListType);
			libvariant::Variant matrixRow1(libvariant::VariantDefines::ListType);
			libvariant::Variant matrixRow2(libvariant::VariantDefines::ListType);
			libvariant::Variant matrixRow3(libvariant::VariantDefines::ListType);
			libvariant::Variant matrixRow4(libvariant::VariantDefines::ListType);

			matrixRow1.Append(matrixData[matrixEntry::r11]);
			matrixRow1.Append(matrixData[matrixEntry::r12]);
			matrixRow1.Append(matrixData[matrixEntry::r13]);
			matrixRow1.Append(matrixData[matrixEntry::x]);

			matrixRow2.Append(matrixData[matrixEntry::r21]);
			matrixRow2.Append(matrixData[matrixEntry::r22]);
			matrixRow2.Append(matrixData[matrixEntry::r23]);
			matrixRow2.Append(matrixData[matrixEntry::y]);

			matrixRow3.Append(matrixData[matrixEntry::r31]);
			matrixRow3.Append(matrixData[matrixEntry::r32]);
			matrixRow3.Append(matrixData[matrixEntry::r33]);
			matrixRow3.Append(matrixData[matrixEntry::z]);

			/* We could also directly put it to 0,0,0,1 but we take the real data
			 * to ease future debugging */
			matrixRow4.Append(matrixData[3]);
			matrixRow4.Append(matrixData[7]);
			matrixRow4.Append(matrixData[11]);
			matrixRow4.Append(matrixData[15]);

			matrix.Append(matrixRow1);
			matrix.Append(matrixRow2);
			matrix.Append(matrixRow3);
			matrix.Append(matrixRow4);

			transformModel.Set("matrix", matrix);
			transformModel.Set("unit", libvariant::Variant("m"));

			transformEntryAsJson.Set("transform", transformModel);
			historyModel.Append(transformEntryAsJson);
		}

		node.Set("history", historyModel);

		return true;
	}

	inline static bool addTransformToJSON(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform, libvariant::Variant& node, std::string transformTag) {

		/* transform data */
		const double* matrixData = transform->getRawData();
		libvariant::Variant transformModel;
		transformModel.Set("type", libvariant::Variant("HomogeneousMatrix44"));

		libvariant::Variant matrix(libvariant::VariantDefines::ListType);
		libvariant::Variant matrixRow1(libvariant::VariantDefines::ListType);
		libvariant::Variant matrixRow2(libvariant::VariantDefines::ListType);
		libvariant::Variant matrixRow3(libvariant::VariantDefines::ListType);
		libvariant::Variant matrixRow4(libvariant::VariantDefines::ListType);

		matrixRow1.Append(matrixData[matrixEntry::r11]);
		matrixRow1.Append(matrixData[matrixEntry::r12]);
		matrixRow1.Append(matrixData[matrixEntry::r13]);
		matrixRow1.Append(matrixData[matrixEntry::x]);

		matrixRow2.Append(matrixData[matrixEntry::r21]);
		matrixRow2.Append(matrixData[matrixEntry::r22]);
		matrixRow2.Append(matrixData[matrixEntry::r23]);
		matrixRow2.Append(matrixData[matrixEntry::y]);

		matrixRow3.Append(matrixData[matrixEntry::r31]);
		matrixRow3.Append(matrixData[matrixEntry::r32]);
		matrixRow3.Append(matrixData[matrixEntry::r33]);
		matrixRow3.Append(matrixData[matrixEntry::z]);

		/* We could also directly put it to 0,0,0,1 but we take the real data
		 * to ease future debugging */
		matrixRow4.Append(matrixData[3]);
		matrixRow4.Append(matrixData[7]);
		matrixRow4.Append(matrixData[11]);
		matrixRow4.Append(matrixData[15]);

		matrix.Append(matrixRow1);
		matrix.Append(matrixRow2);
		matrix.Append(matrixRow3);
		matrix.Append(matrixRow4);

		transformModel.Set("matrix", matrix);
		transformModel.Set("unit", libvariant::Variant("m"));

		node.Set(transformTag, transformModel);

		return true;
	}

	inline static bool getShapeFromJSON(brics_3d::rsg::Shape::ShapePtr& shape, libvariant::Variant& node, string shapeTag = "geometry") {

		brics_3d::rsg::Sphere::SpherePtr newSphere;
		brics_3d::rsg::Cylinder::CylinderPtr newCylinder;
		brics_3d::rsg::Box::BoxPtr newBox;
		brics_3d::Units::DistanceUnit unit;

		if(node.Contains("unit")) {
			LOG(DEBUG) << "JSONTypecaster: GeometricNode has the following unit: " << node.Get("unit").AsString();
			string unitAsString = node.Get("unit").AsString();
			toDistanceUnit(unitAsString, unit);
		}

		if(node.Contains("geometry")) {
			LOG(DEBUG) << "JSONTypecaster: GeometricNode has the following geometry:";

			libvariant::Variant geometry = node.Get(shapeTag);

			if(geometry.Contains("@geometrytype")) {
				string geometrytype = geometry.Get("@geometrytype").AsString();

				if(geometrytype.compare("Sphere") == 0) {
					LOG(DEBUG) << "\t\t found a Sphere.";
					if(!geometry.Contains("radius")) { LOG(ERROR) << "\t\t radius is missing"; return false;};

					newSphere = brics_3d::rsg::Sphere::SpherePtr(new brics_3d::rsg::Sphere());
					newSphere->setRadius(Units::distanceToMeters(geometry.Get("radius").AsDouble(), unit));
					shape = newSphere;

				} else if (geometrytype.compare("Cylinder") == 0) {
					LOG(DEBUG) << "\t\t found a Cylinder.";
					if(!geometry.Contains("radius")) { LOG(ERROR) << "\t\t radius is missing"; return false;};
					if(!geometry.Contains("height")) { LOG(ERROR) << "\t\t height is missing"; return false;};

					newCylinder = brics_3d::rsg::Cylinder::CylinderPtr(new brics_3d::rsg::Cylinder());
					LOG (DEBUG) << "Units::distanceToMeters(geometry.Get(radius).AsDouble(), unit) " << Units::distanceToMeters(geometry.Get("radius").AsDouble(), unit);
					newCylinder->setRadius(Units::distanceToMeters(geometry.Get("radius").AsDouble(), unit));
					newCylinder->setHeight(Units::distanceToMeters(geometry.Get("height").AsDouble(), unit));
					shape = newCylinder;

				} else if (geometrytype.compare("Box") == 0) {
					LOG(DEBUG) << "\t\t found a Box.";
					if(!geometry.Contains("sizeX")) { LOG(ERROR) << "\t\t sizeX is missing"; return false;};
					if(!geometry.Contains("sizeY")) { LOG(ERROR) << "\t\t sizeY is missing"; return false;};
					if(!geometry.Contains("sizeZ")) { LOG(ERROR) << "\t\t sizeZ is missing"; return false;};

					newBox = brics_3d::rsg::Box::BoxPtr(new brics_3d::rsg::Box());
					newBox->setSizeX(Units::distanceToMeters(geometry.Get("sizeX").AsDouble(), unit));
					newBox->setSizeY(Units::distanceToMeters(geometry.Get("sizeY").AsDouble(), unit));
					newBox->setSizeZ(Units::distanceToMeters(geometry.Get("sizeZ").AsDouble(), unit));
					shape = newBox;

				} else if (geometrytype.compare("PointCloud3D") == 0) {
					LOG(DEBUG) << "\t\t found a PointCloud3D.";
				} else if (geometrytype.compare("PointCloud3DBinaryBlob") == 0) {
					LOG(DEBUG) << "\t\t found a PointCloud3DBinaryBlob.";
				} else if (geometrytype.compare("TriangleMesh3D") == 0) {
					LOG(DEBUG) << "\t\t found a TriangleMesh3D.";

//					brics_3d::ITriangleMesh::ITriangleMeshPtr newMesh(new brics_3d::TriangleMeshImplicit());
					brics_3d::TriangleMeshImplicit::TriangleMeshImplicitPtr newMesh(new brics_3d::TriangleMeshImplicit());
					brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr newMeshContainer(new brics_3d::rsg::Mesh<brics_3d::ITriangleMesh>());
					newMeshContainer->data = newMesh;
					std::vector<Point3D>* points = newMesh->getVertices();
					std::vector<int>* indices = newMesh->getIndices();

					if(!geometry.Contains("points")) { LOG(ERROR) << "\t\t points list is missing"; return false;};
					libvariant::Variant pointList = geometry.Get("points");
					if (pointList.IsList()) {
						for (libvariant::Variant::ListIterator i(pointList.ListBegin()), e(pointList.ListEnd()); i!=e; ++i) {

							if(!i->Contains("@pointtype")) { LOG(ERROR) << "\t\t @pointtype list is missing"; return false;};

							if(i->Get("@pointtype").AsString().compare("Point3D") == 0) {

								points->push_back(Point3D(
										Units::distanceToMeters(i->Get("x").AsDouble(), unit),
										Units::distanceToMeters(i->Get("y").AsDouble(), unit),
										Units::distanceToMeters(i->Get("z").AsDouble(), unit)
								));

								LOG(DEBUG) << "\t\t Point3D = " << points->back();

							} else {
								LOG(WARNING) << "JSONTypecaster: unknown @geometrytype: " << i->Get("@geometrytype").AsString() ;
							}

						}
					}

					if(!geometry.Contains("indices")) { LOG(ERROR) << "\t\t indices list is missing"; return false;};
					libvariant::Variant indexList = geometry.Get("indices");
					if (indexList.IsList()) {
						for (libvariant::Variant::ListIterator i(indexList.ListBegin()), e(indexList.ListEnd()); i!=e; ++i) {

							indices->push_back(i->AsInt());
							LOG(DEBUG) << "\t\t Index = " << i->AsInt();
						}
					}

					shape = newMeshContainer;

				} else {
					LOG(ERROR) << "JSONTypecaster: unkonwn geometry type.";
					return false;
				}

			} else {
				LOG(ERROR) << "JSONTypecaster: GeometricNode has no following @geometrytype contained on geometry:";
				return false;
			}
		}

		if(shape == 0) {
			return false;
		}

		return true;
	}

	inline static bool addShapeToJSON(brics_3d::rsg::Shape::ShapePtr& shape, libvariant::Variant& node, string shapeTag) {
		LOG(DEBUG) << "JSONTypecaster: addShapeToJSON: ";

		rsg::Sphere::SpherePtr sphere(new rsg::Sphere());
		sphere =  boost::dynamic_pointer_cast<rsg::Sphere>(shape);
		rsg::Box::BoxPtr box(new rsg::Box());
		box =  boost::dynamic_pointer_cast<rsg::Box>(shape);
		rsg::Cylinder::CylinderPtr cylinder(new rsg::Cylinder());
		cylinder =  boost::dynamic_pointer_cast<rsg::Cylinder>(shape);
		rsg::Mesh<brics_3d::ITriangleMesh>::MeshPtr mesh(new rsg::Mesh<brics_3d::ITriangleMesh>());
		mesh = boost::dynamic_pointer_cast<rsg::Mesh<brics_3d::ITriangleMesh> >(shape);

		libvariant::Variant geometry;

		if (sphere !=0) {
			LOG(DEBUG) << "                 -> Found a sphere.";

			geometry.Set("@geometrytype", libvariant::Variant("Sphere"));
			geometry.Set("radius", libvariant::Variant(sphere->getRadius()));
			node.Set(shapeTag, geometry);

		} else if (cylinder !=0) {
			LOG(DEBUG) << "                 -> Found a cylinder.";

			geometry.Set("@geometrytype", libvariant::Variant("Cylinder"));
			geometry.Set("radius", libvariant::Variant(cylinder->getRadius()));
			geometry.Set("height", libvariant::Variant(cylinder->getHeight()));
			node.Set(shapeTag, geometry);

		} else if (box !=0) {
			LOG(DEBUG) << "                 -> Found a box.";

			geometry.Set("@geometrytype", libvariant::Variant("Box"));
			geometry.Set("sizeX", libvariant::Variant(box->getSizeX()));
			geometry.Set("sizeY", libvariant::Variant(box->getSizeY()));
			geometry.Set("sizeZ", libvariant::Variant(box->getSizeZ()));
			node.Set(shapeTag, geometry);

		} else if (shape->getPointCloudIterator() != 0) {
			LOG(DEBUG) << "                 -> Found a point cloud.";

			geometry.Set("@geometrytype", libvariant::Variant("PointCloud3D"));

			libvariant::Variant points;

			IPoint3DIterator::IPoint3DIteratorPtr it = shape->getPointCloudIterator();
			for (it->begin(); !it->end(); it->next()) {
				libvariant::Variant point;

				point.Set("x", libvariant::Variant(it->getX()));
				point.Set("y", libvariant::Variant(it->getY()));
				point.Set("z", libvariant::Variant(it->getZ()));

				if(it->getRawData()->asColoredPoint3D() != 0) {
					geometry.Set("@pointtype", libvariant::Variant("ColoredPoint3D"));
					point.Set("r", libvariant::Variant(it->getRawData()->asColoredPoint3D()->getR()));
					point.Set("g", libvariant::Variant(it->getRawData()->asColoredPoint3D()->getG()));
					point.Set("b", libvariant::Variant(it->getRawData()->asColoredPoint3D()->getB()));
				} else {
					geometry.Set("@pointtype", libvariant::Variant("Point3D"));
				}

				points.Append(point);
			}

			geometry.Set("points", points);
			node.Set(shapeTag, geometry);

		} else if (mesh != 0) {
			LOG(DEBUG) << "                 -> Found a mesh.";

			geometry.Set("@geometrytype", libvariant::Variant("TriangleMesh3D"));

			libvariant::Variant points;
			libvariant::Variant indices(libvariant::VariantDefines::ListType);

			TriangleMeshImplicit* meshImplicit = dynamic_cast<TriangleMeshImplicit*>(mesh.get());
			if (meshImplicit != NULL) { //here we exploit knowledge about the implicit triangle representation

				LOG(DEBUG) << "              	   -> It is a TriangleMeshImplicit.";
				std::vector<brics_3d::Point3D>::const_iterator it;
				for (it = meshImplicit->getVertices()->begin(); it != meshImplicit->getVertices()->end(); ++it) {

					libvariant::Variant point;
					point.Set("x", libvariant::Variant(it->getX()));
					point.Set("y", libvariant::Variant(it->getY()));
					point.Set("z", libvariant::Variant(it->getZ()));
					point.Set("@pointtype", libvariant::Variant("Point3D"));
					points.Append(point);
				}

				for (std::vector<int>::const_iterator it = meshImplicit->getIndices()->begin(); it != meshImplicit->getIndices()->end(); ++it) {
					indices.Append(*it);
				}
			} else { // here we go with the generic (possible less efficient) representation

					Point3D tmpVertex;
					int indexCount = 0;

					for (int i =0 ; i < mesh->data->getSize(); ++i) { // loop over all triangles
						for (int j = 0; j <= 2; ++j) { // loop over the three vertices per triangle

							/* vertex/point */
							tmpVertex = *mesh->data->getTriangleVertex(i,j);
							libvariant::Variant point;
							point.Set("x", libvariant::Variant(tmpVertex.getX()));
							point.Set("y", libvariant::Variant(tmpVertex.getY()));
							point.Set("z", libvariant::Variant(tmpVertex.getZ()));
							point.Set("@pointtype", libvariant::Variant("Point3D"));
							points.Append(point);

							/* index */
							indices.Append(indexCount); //one-to-one mapping
							indexCount++;

						}
					}

			}

			geometry.Set("points", points);
			geometry.Set("indices", indices);
			node.Set(shapeTag, geometry);

		} else {
			LOG(ERROR) << "JSONTypecaster: addShapeToJSON: Shape type not yet supported.";
			return false;
		}


		return true;
	}

	inline static bool toDistanceUnit(string stringAsUnit, Units::DistanceUnit& unit) {
		if (stringAsUnit.compare("nm") == 0) {
			unit = Units::NanoMeter;
		} else if (stringAsUnit.compare("um") == 0) {
			unit = Units::MicroMeter;
		} else if (stringAsUnit.compare("mm") == 0) {
			unit = Units::MilliMeter;
		} else if (stringAsUnit.compare("cm") == 0) {
			unit = Units::CentiMeter;
		} else if (stringAsUnit.compare("dm") == 0) {
			unit = Units::DeciMeter;
		} else if (stringAsUnit.compare("m") == 0) {
			unit = Units::Meter;
		} else if (stringAsUnit.compare("km") == 0) {
			unit = Units::KiloMeter;
		} else {
			LOG(WARNING) << "JSONTypecaster: toDistanceUnit: unrecognized type:" << stringAsUnit;
			// Fallback is meter:
			unit = Units::Meter;
			return false;
		}

		LOG(DEBUG) << "JSONTypecaster: unit = " << stringAsUnit << " " << unit;

		return true;
	}

};

} /* namespace rsg */
} /* namespace brics_3d */



#endif /* RSG_JSONTYPECASTER_H_ */

/* EOF */
