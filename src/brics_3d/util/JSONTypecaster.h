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
#include "brics_3d/worldModel/WorldModel.h"
#include <Variant/Variant.h>

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


	inline static rsg::Id getIdFromJSON(libvariant::Variant& node, string idTag) {
		rsg::Id id = 0; //NiL

		if(node.Contains(idTag)) { // required
			id.fromString(node.Get(idTag).AsString());
			assert(!id.isNil());
			LOG(DEBUG) << "JSONTypecaster: Id " << idTag << " is = " << id;
		} else {
			LOG(ERROR) << "Can not parse model. No node Id " << idTag << " specified.";
		}

		return id;
	}

	inline static bool addIdToJSON(brics_3d::rsg::Id id, libvariant::Variant& node, string idTag) {

		node.Set(idTag, libvariant::Variant(id.toString()));

		return true;
	}

	inline static vector<rsg::Id> getIdsFromJSON(libvariant::Variant& node, string idsTag) {
		vector<rsg::Id> ids;

		LOG(DEBUG) << "JSONTypecaster: Id list " << idsTag << " has the following Ids:";
		libvariant::Variant idList = node.Get(idsTag);
		if (idList.IsList()) {
			for (libvariant::Variant::ListIterator i(idList.ListBegin()), e(idList.ListEnd()); i!=e; ++i) {
				Id id;
				id.fromString(i->AsString());
				assert(!id.isNil());
				LOG(DEBUG) << "JSONTypecaster: \t " << id;
				ids.push_back(id);
			}
		}

		return ids;
	}

	inline static rsg::TimeStamp getTimeStampFromJSON(libvariant::Variant& node, string stampTag) {
		rsg::TimeStamp stamp(0); //-inf?

		if(node.Contains(stampTag)) {
			LOG(DEBUG) << "JSONTypecaster: entity has the following time stamp:";
			libvariant::Variant stampModel = node.Get(stampTag);

			if(stampModel.Contains("@stamptype")) {
				assert(stampModel.Contains("stamp"));

				if( stampModel.Get("@stamptype").AsString().compare("TimeStampUTCms") == 0 ) {

					stamp = TimeStamp( stampModel.Get("stamp").AsDouble(), Units::MilliSecond);

				} else if ( stampModel.Get("@stamptype").AsString().compare("TimeStampDate") == 0 ) {

					string dateTime =  stampModel.Get("stamp").AsString();
					struct tm tmlol;
					strptime(dateTime.c_str(), "%Y-%m-%dT%H:%M:%SZ", &tmlol);
					time_t t = mktime(&tmlol); // Seconds since epoche
					stamp = TimeStamp(t, Units::Second);

				} else {
					LOG(ERROR) << "JSONTypecaster: Time stamp defeinition has unknown type @stamptype identifier";
				}

			} else {
				LOG(ERROR) << "JSONTypecaster: Time stamp fefinition has no type @stamptype identifier";
			}
		} else {
			LOG(ERROR) << "JSONTypecaster: entity has no stamp tag: " << stampTag;
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

	inline static std::vector<rsg::Attribute> getAttributesFromJSON (libvariant::Variant& node) {
		std::vector<rsg::Attribute> attributes;
		attributes.clear();

		if(node.Contains("attributes")) {
			LOG(DEBUG) << "JSONTypecaster: Node has the following attributes:";

			libvariant::Variant attributeList = node.Get("attributes");
			if (attributeList.IsList()) {
				for (libvariant::Variant::ListIterator i(attributeList.ListBegin()), e(attributeList.ListEnd()); i!=e; ++i) {
					assert(i->Contains("key"));
					assert(i->Contains("value"));

					try {

						LOG(DEBUG) << "\t( " << i->Get("key").AsString() << ", " << i->Get("value").AsString() << " )";
						attributes.push_back(rsg::Attribute(i->Get("key").AsString(), i->Get("value").AsString()));

					/*
					 * Here we are rather permissive on what exactly a value is.
					 * We actually allow for a complete JSON model here.
					 */
					} catch (libvariant::UnableToConvertError e) {

						LOG(DEBUG) << "JSONTypecaster: parsing a non string value";
						string complexValue = libvariant::Serialize(*i, libvariant::SERIALIZE_JSON);

						LOG(DEBUG) << "\t( " << i->Get("key").AsString() << ", " << complexValue << " )";
						attributes.push_back(rsg::Attribute(i->Get("key").AsString(), complexValue));
					}
				}
			}
		}

		return attributes;
	}

	inline static bool addAttributesToJSON(std::vector<brics_3d::rsg::Attribute> attributes, libvariant::Variant& node) {
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

		node.Set("attributes", attributesAsJson);

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

							assert(i->Contains("transform"));
							libvariant::Variant transformModel = i->Get("transform");
							assert(transformModel.Contains("type"));

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

	inline static bool getShapeFromJSON(brics_3d::rsg::Shape::ShapePtr& shape, libvariant::Variant& node) {

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

			libvariant::Variant geometry = node.Get("geometry");

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
				} else {
					LOG(ERROR) << "JSONTypecaster: unkonwn geometry type.";
					return false;
				}

			} else {
				LOG(ERROR) << "JSONTypecaster: GeometricNode has no following @geometrytype contained on geometry:";
				return false;
			}
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
