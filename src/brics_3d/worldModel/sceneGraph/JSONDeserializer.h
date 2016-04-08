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

#ifndef RSG_JSONDESERIALIZER_H_
#define RSG_JSONDESERIALIZER_H_

#include "brics_3d/util/JSONTypecaster.h"
#include "brics_3d/worldModel/WorldModel.h"
#include "brics_3d/worldModel/sceneGraph/IPort.h"

namespace brics_3d {
namespace rsg {



class JSONDeserializer : public IInputPort  {
public:
	/**
	 * @brief Default constructor.
	 * @param wm The WorldModel to be updated.
	 */
	JSONDeserializer(WorldModel* wm);

	/**
	 * @brief Constructor to be used with intermediate filters
	 * @param wm The WorldModel for querying the root Id and time.
	 * @param sceneUpdater The scene to be updated. Typically an update filter.
	 */
	JSONDeserializer(WorldModel* wm, ISceneGraphUpdate* sceneUpdater);

	virtual ~JSONDeserializer();

	int write(const char *dataBuffer, int dataLength, int &transferredBytes);
	int write(std::string data);
	int write(libvariant::Variant& model);

	bool isMapUnknownParentIdsToRootId() const {
		return mapUnknownParentIdsToRootId;
	}

	void setMapUnknownParentIdsToRootId(bool mapUnknownParentIdsToRootId) {
		this->mapUnknownParentIdsToRootId = mapUnknownParentIdsToRootId;
	}

	static Id getRootIdFromJSONModel(std::string data);

private:

	virtual bool handleWorldModelUpdate(libvariant::Variant& model);
	virtual bool handleWorldModelAgent(libvariant::Variant& model);
	virtual bool handleGraphPrimitive(libvariant::Variant& atom, rsg::Id parentId);
	virtual bool handleChilden(libvariant::Variant& group, rsg::Id parentId);
	virtual bool handleConnections(libvariant::Variant& group, rsg::Id parentId);

	virtual bool doAddNode(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doAddGroup(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doAddTransformNode(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doAddGeometricNode(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doAddRemoteRootNode(libvariant::Variant& group);
	virtual bool doAddConnection(libvariant::Variant& connection, rsg::Id parentId);
	virtual bool doSetNodeAttributes(libvariant::Variant& group);
	virtual bool doSetTransform(libvariant::Variant& group);
	virtual bool doDeleteNode(libvariant::Variant& group);
	virtual bool doAddParent(libvariant::Variant& group, rsg::Id parentId);
	virtual bool doRemoveParent(libvariant::Variant& group, rsg::Id parentId);

	WorldModel* wm;
	ISceneGraphUpdate* sceneUpdater;
	bool mapUnknownParentIdsToRootId;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_JSONDESERIALIZER_H_ */

/* EOF */
