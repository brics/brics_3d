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

#ifndef RSG_UPDATESTOSCENEGRAPHLISTENER_H_
#define RSG_UPDATESTOSCENEGRAPHLISTENER_H_

#include <brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h>
#include <brics_3d/worldModel/sceneGraph/Attribute.h>
#include <brics_3d/util/Timer.h>

namespace brics_3d {
namespace rsg {

class UpdatesToSceneGraphListener : public ISceneGraphUpdateObserver {
public:
	UpdatesToSceneGraphListener();
	virtual ~UpdatesToSceneGraphListener();

	/* implemetntations of observer interface */
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

    bool attachSceneGraph(ISceneGraphUpdate* observer);
    bool detachSceneGraph(ISceneGraphUpdate* observer);

	bool isForcedIdPolicy() const {
		return forcedIdPolicy;
	}

	void setForcedIdPolicy(bool forcedIdPolicy) {
		this->forcedIdPolicy = forcedIdPolicy;
	}

private:

    /**
     * Set of observers that will be notified.
     * Note this is not the observer intarface ISceneGraphUpdateObserver rather than ISceneGraphUpdate to allow
     * to directly attach another scene graph by it SceneGraphFacade.
     */
    std::vector<ISceneGraphUpdate*> updateObservers;

    /**
     * Here we add the configurable policy of enforcing to take over the IDs or not.
     * The is the major difference between this implementation and a plain ISceneGraphUpdateObserver.
     * Default value is true in order to create an exact copy of the observed scene.
     * A value of false causes to create an identical scene graph but with freshly generated IDs, thus
     * having two different graphs. The latter could be useful to multiply a particular in a scene structure.
     */
    bool forcedIdPolicy;

    /// Default policy @see forcedIdPolicy
    const static bool ENFORCE_TO_TAKE_OVER_EXISTING_IDS = true;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_UPDATESTOSCENEGRAPHLISTENER_H_ */

/* EOF */
