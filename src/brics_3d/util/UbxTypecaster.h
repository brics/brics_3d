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

#ifndef UBXTYPECASTER_H_
#define UBXTYPECASTER_H_

#include <vector>
#include <brics_3d/core/Logger.h>
#include "brics_3d/worldModel/sceneGraph/Id.h"
//#include "/opt/src/sandbox/brics_3d_function_blocks/types/rsg/types/rsg_types.h" //FIXME
//#include "rsg_ubx/types/rsg_types.h" // external folder

namespace brics_3d {
namespace rsg {

/**
 * @brief Helper class to convert from scene graph types to microblox types.
 *
 * This involves in particular conversion forth and back of set of ids.
 * @ingroup sceneGraph
 */
class UbxTypecaster {
public:
	UbxTypecaster(){};
	virtual ~UbxTypecaster(){};

	inline static void convertIdToUbx(brics_3d::rsg::Id id,  rsg_uuid& convertedId) {
		LOG(DEBUG) << "UbxTypecaster: Converting new Id to Ubx [->]: " << id;
		Uuid::const_iterator i_data = id.begin();
		for (size_t i = 0; i < Uuid::arraySize; ++i, ++i_data) {
			convertedId.data[i] = *i_data;
		}
	}

	inline static void convertIdFromUbx(rsg_uuid id, brics_3d::rsg::Id& convertedId) {
		brics_3d::rsg::Uuid::iterator i_data = convertedId.begin();
		for (size_t i = 0; i < brics_3d::rsg::Uuid::arraySize; ++i, ++i_data) {
			*i_data = id.data[i];
		}
		LOG(DEBUG) << "UbxTypecaster: Converting new Id from Ubx [<-]: "  << convertedId;
	}

	inline static void convertIdsToUbx(std::vector<brics_3d::rsg::Id> ids,	rsg_ids& convertedIds) {
//		convertedIds.resize(ids.size());
//		assert(ids.size() <= MAX_NUMBER_OF_IDS);
		if (ids.size() > MAX_NUMBER_OF_IDS) {
			LOG(ERROR) << "UbxTypecaster: ids.numberOfIds > MAX_NUMBER_OF_IDS: " << ids.size();
			return;
		}

		convertedIds.numberOfIds = static_cast<unsigned int>(ids.size());
		for (unsigned int i = 0; i < static_cast<unsigned int>(ids.size()); ++i) {
			convertIdToUbx(ids[i], convertedIds.ids[i]); // in-place conversion
		}
	}

	inline static void convertIdsFromUbx(rsg_ids  ids, std::vector<brics_3d::rsg::Id>& convertedIds) {
		//assert(ids.numberOfIds <= MAX_NUMBER_OF_IDS);
		if (ids.numberOfIds > MAX_NUMBER_OF_IDS) {
			LOG(ERROR) << "UbxTypecaster: ids.numberOfIds > MAX_NUMBER_OF_IDS: " << ids.numberOfIds;
			return;
		}

		convertedIds.resize(static_cast<size_t>(ids.numberOfIds));
		for (unsigned int i = 0; i < static_cast<unsigned int>(ids.numberOfIds); ++i) {
			convertIdFromUbx(ids.ids[i], convertedIds[i]); // in-place conversion
		}
	}

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* UBXTYPECASTER_H_ */

/* EOF */
