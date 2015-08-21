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


	inline static Id getIdFromJSON(libvariant::Variant& node, string idTag) {
		rsg::Id id = 0; //NiL

		if(node.Contains(idTag)) { // required
			id.fromString(node.Get(idTag).AsString());
			assert(!id.isNil());
			LOG(DEBUG) << "Id " << idTag << " is = " << id;
		} else {
			LOG(ERROR) << "Can not parse model. No node Id " << idTag << " specified.";
		}

		return id;
	}

	inline static std::vector<rsg::Attribute> getAttributesFromJSON (libvariant::Variant& node) {
		std::vector<rsg::Attribute> attributes;
		attributes.clear();

		if(node.Contains("attributes")) {
			LOG(DEBUG) << "Node has the following attributes:";

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
};

} /* namespace rsg */
} /* namespace brics_3d */



#endif /* RSG_JSONTYPECASTER_H_ */

/* EOF */
