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

#ifndef RSG_JSONSERIALIZER_H_
#define RSG_JSONSERIALIZER_H_

#include "brics_3d/core/Logger.h"
#include "brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h"
#include "brics_3d/worldModel/sceneGraph/IPort.h"

#include <Variant/Variant.h>

namespace brics_3d {
namespace rsg {

class JSONSerializer : public ISceneGraphUpdateObserver {
public:
	JSONSerializer(IOutputPort* port);
	virtual ~JSONSerializer();

	bool addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId = false);
	bool addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forcedId = false);
	bool addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId = false);
	bool addRemoteRootNode(Id rootId, vector<Attribute> attributes);
	bool addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId = false);
	bool setNodeAttributes(Id id, vector<Attribute> newAttributes, TimeStamp timeStamp = TimeStamp(0));
	bool setTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
	bool setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp);
	bool deleteNode(Id id);
	bool addParent(Id id, Id parentId);
	bool removeParent(Id id, Id parentId);

	bool getStoreMessageBackupsOnFileSystem() const {
		return storeMessageBackupsOnFileSystem;
	}

	void setStoreMessageBackupsOnFileSystem(
			bool storeMessageBackupsOnFileSystem) {
		this->storeMessageBackupsOnFileSystem = storeMessageBackupsOnFileSystem;
	}

private:

	IOutputPort* port;
	bool storeMessageBackupsOnFileSystem;
	std::string fileSuffix;

protected:

	bool doSendMessage(libvariant::Variant& message);
	bool doSendMessage(std::string& message);

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_JSONSERIALIZER_H_ */

/* EOF */
