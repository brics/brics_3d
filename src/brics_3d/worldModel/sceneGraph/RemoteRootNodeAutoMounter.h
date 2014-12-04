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

#ifndef RSG_REMOTEROOTNODEAUTOMOUNTER_H_
#define RSG_REMOTEROOTNODEAUTOMOUNTER_H_

#include <brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h>
#include <brics_3d/worldModel/sceneGraph/SceneGraphFacade.h>

namespace brics_3d {
namespace rsg {

/**
 * @brief Automatically connects new remote nodes to the scene graph.
 *
 * This class observes all changes to the romote root nodes
 * i.e. invocations of addRemoteRootNode().
 * For each new remote node a new parent-child relation will be added,
 * whereas the node as specifies by mountPoint serves as parent.
 *
 * This is somewhat similar to the fstab file in a Linux like system.
 */
class RemoteRootNodeAutoMounter : public ISceneGraphUpdateObserver {
public:

	/**
	 * @brief Constructor with locl root node as default mount point.
	 * @param observedScene Pointer to the observed scene. Required to
	 *        automitically add a parent child relation to it.
	 *
	 */
	RemoteRootNodeAutoMounter(SceneGraphFacade* observedScene);

	/**
	 * @brief Constructor with abitrary mount point.
	 * @param observedScene Pointer to the observed scene. Required to
	 *        automitically add a parent child relation to it.
	 * @param mountPoint Any new remote root node will be added as child to
	 *        this node.
	 */
	RemoteRootNodeAutoMounter(SceneGraphFacade* observedScene, Id mountPoint);
	virtual ~RemoteRootNodeAutoMounter();

	/* implemetntations of observer interface */
	bool addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
	bool addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId = false);
    bool addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forcedId = false);
	bool addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId = false);
	bool addRemoteRootNode(Id rootId, vector<Attribute> attributes);
	bool setNodeAttributes(Id id, vector<Attribute> newAttributes);
	bool setTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
    bool setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp);
	bool deleteNode(Id id);
	bool addParent(Id id, Id parentId);
    bool removeParent(Id id, Id parentId);

private:

    Id mountPoint;

    SceneGraphFacade* observedScene;

};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_REMOTEROOTNODEAUTOMOUNTER_H_ */

/* EOF */
