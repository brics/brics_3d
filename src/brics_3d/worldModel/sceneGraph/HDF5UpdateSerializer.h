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

#ifndef RSG_HDF5UPDATESERIALIZER_H_
#define RSG_HDF5UPDATESERIALIZER_H_

#include "brics_3d/core/Logger.h"
#include "brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h"
#include "brics_3d/util/HDF5Typecaster.h"

namespace brics_3d {
namespace rsg {


class IOutputPort {
public:
	IOutputPort(){};
	virtual ~IOutputPort(){};
	virtual int write(const char *dataBuffer, int dataLength, int &transferredBytes) = 0;
};

class HDF5UpdateSerializer : public ISceneGraphUpdateObserver {
public:
	HDF5UpdateSerializer(IOutputPort* port);
	virtual ~HDF5UpdateSerializer();

	bool addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId = false);
	bool addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forcedId = false);
	bool addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId = false);
	bool addRemoteRootNode(Id rootId, vector<Attribute> attributes);
	bool addConnection(Id parentId, Id& assignedId, vector<Attribute> attributes, vector<Id> sourceIds, vector<Id> targetIds, TimeStamp start, TimeStamp end, bool forcedId = false);
	bool setNodeAttributes(Id id, vector<Attribute> newAttributes);
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

protected:

	bool doSendMessage(std::string messageName);
	bool doSendMessage(H5::H5File message);

	IOutputPort* port;
	bool storeMessageBackupsOnFileSystem;
	size_t fileImageIncremet;

	std::string fileSuffix;
};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_HDF5UPDATESERIALIZER_H_ */

/* EOF */
