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

				} else {
					LOG(ERROR) << "JSONTypecaster: Time stamp defeinition has unknown type @stamptype identifier";
				}

			} else {
				LOG(ERROR) << "JSONTypecaster: Time stamp fefinition has no type @stamptype identifier";
			}
		} else {
			LOG(ERROR) << "JSONTypecaster: entity has no stamp tag: " << stampTag;
		}



		return stamp;
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
					LOG(DEBUG) << "\t( " << i->Get("key").AsString() << ", " << i->Get("value").AsString() << " )";
					attributes.push_back(rsg::Attribute(i->Get("key").AsString(), i->Get("value").AsString()));
				}
			}
		}

		return attributes;
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
};

} /* namespace rsg */
} /* namespace brics_3d */



#endif /* RSG_JSONTYPECASTER_H_ */

/* EOF */
