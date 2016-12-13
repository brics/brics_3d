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

#ifndef RSG_TIMESTAMPER_H_
#define RSG_TIMESTAMPER_H_

#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h>
#include <brics_3d/worldModel/sceneGraph/Attribute.h>
#include <brics_3d/util/Benchmark.h>

namespace brics_3d {
namespace rsg {

class TimeStamper : public ISceneGraphUpdateObserver {

	enum COMMAND {
		ADD_NODE = 0,
		ADD_GROUP = 1,
		ADD_TRANSFORM_NODE = 2,
		ADD_UNCERTAIN_TRANSFORM_NODE = 3,
		ADD_GEOMETRIC_NODE = 4,
		SET_NODE_ATTRIBUTES = 5,
		SET_TRANSFORM = 6,
		SET_UNCERTAIN_TRANSFORM = 7,
		DELETE_NODE = 8,
		ADD_PARENT = 9,
		REMOVE_PARENT = 10
	};

public:
	TimeStamper(WorldModel* wmHandle, std::string name);
	virtual ~TimeStamper();

	/* implemetntations of observer interface */
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

    bool attachUpdateObserver(ISceneGraphUpdateObserver* observer);
    bool detachUpdateObserver(ISceneGraphUpdateObserver* observer);


private:

    /// Handle to world model. Used for the timer.
    WorldModel* wm;

    /// Logger for the timestamps
    Benchmark* timeStampLog;

    /// Set of observers that will be notified when the update function will be called and the LoD contraints are satisfied.
    std::vector<ISceneGraphUpdateObserver*> updateObservers;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_TIMESTAMPER_H_ */

/* EOF */
