/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
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

#ifndef RSG_SCENEGRAPHFACADE_H
#define RSG_SCENEGRAPHFACADE_H

#include "ISceneGraphQuery.h"
#include "ISceneGraphUpdate.h"
#include "ISceneGraphUpdateObserver.h"
#include "IIdGenerator.h"
#include "Group.h"
#include "Node.h"
#include "Transform.h"
#include "UncertainTransform.h"
#include "GeometricNode.h"
#include "Shape.h"

#include <map>
#include <boost/weak_ptr.hpp>
using std::map;


namespace brics_3d {

namespace rsg {

/**
 * @brief The central handle to create and maintain a robot scenegraph. It holds the root node of the scene graph.
 *
 * The SceneGraphFacade takes care (maintains consistency) of mapping between IDs and internal pointers.
 * The implemented interfaces allow to create and maintain a scengraph bases on the node IDs only.
 *
 * @ingroup sceneGraph
 */
class SceneGraphFacade : public ISceneGraphQuery, public ISceneGraphUpdate {

  public:

	/**
	 * @brief Default constructor.
	 * Uses the UuidGenerator as default.
	 */
	SceneGraphFacade();

	/**
	 * @brief Constructor with specific Id generator.
	 * @param idGenerator Handle to generator that will be used.
	 *        Ownership is passed to this class!
	 */
    SceneGraphFacade(IIdGenerator* idGenerator);

    /**
     * @brief Default destructor.
     */
    virtual ~SceneGraphFacade();

    /* Facade specific methods */
    Id getRootId();
    bool addRemoteRootNode(Id rootId, vector<Attribute> attributes);
    bool getRemoteRootNodes(vector<Id>& ids);

    /* Implemented query interfaces */
    bool getNodes(vector<Attribute> attributes, vector<Id>& ids); //subgraph?
    bool getNodeAttributes(Id id, vector<Attribute>& attributes);
    bool getNodeParents(Id id, vector<Id>& parentIds);
    bool getGroupChildren(Id id, vector<Id>& childIds);
    bool getTransform(Id id, TimeStamp timeStamp, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform);
    bool getUncertainTransform(Id id, TimeStamp timeStamp, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform, ITransformUncertainty::ITransformUncertaintyPtr &uncertainty);
    bool getGeometry(Id id, Shape::ShapePtr& shape, TimeStamp& timeStamp);

    bool getTransformForNode (Id id, Id idReferenceNode, TimeStamp timeStamp, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr& transform);

    /* Implemented update interfaces */
    bool addNode(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
    bool addGroup(Id parentId, Id& assignedId, vector<Attribute> attributes, bool forcedId = false);
    bool addTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp, bool forcedId = false);
    bool addUncertainTransformNode(Id parentId, Id& assignedId, vector<Attribute> attributes, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp, bool forcedId = false);
    bool addGeometricNode(Id parentId, Id& assignedId, vector<Attribute> attributes, Shape::ShapePtr shape, TimeStamp timeStamp, bool forcedId = false);
    bool setNodeAttributes(Id id, vector<Attribute> newAttributes);
    bool setTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, TimeStamp timeStamp);
    bool setUncertainTransform(Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform, ITransformUncertainty::ITransformUncertaintyPtr uncertainty, TimeStamp timeStamp);
	bool deleteNode(Id id);
    bool addParent(Id id, Id parentId);
    bool removeParent(Id id, Id parentId);

    /* Configuration */
    bool attachUpdateObserver(ISceneGraphUpdateObserver* observer);
    bool detachUpdateObserver(ISceneGraphUpdateObserver* observer);

	bool isCallObserversEvenIfErrorsOccurred() const;
	void setCallObserversEvenIfErrorsOccurred(bool callObserversEvenIfErrorsOccurred);

    /* Coordination methods */
	bool executeGraphTraverser(INodeVisitor* visitor, Id subgraphId);

  private:

	/// Internal initiaization.
    void initialize();

    /**
     * @brief Resolved IDs to references.
     * @param id The ID.
     * @return The reference. Will be NULL in case no reference could be found.
     */
    Node::NodeWeakPtr findNodeRecerence(Id id);

    /**
     * @brief Test if an ID is in the idLookUpTable.
     * @param id The ID.
     * @return True if ID is in table, otherwise false.
     */
    bool doesIdExist(Id id);

    /// The root of all evil...
    Group::GroupPtr rootNode;

    /// Possible set of remote root nodes.
    vector<Group::GroupPtr> remoteRootNodes;

    /// Table that maps IDs to references.
    map<Id, Node::NodeWeakPtr > idLookUpTable;

    /// Iterator for idLookUpTable
    map<Id, Node::NodeWeakPtr >::const_iterator nodeIterator;

    /// Handle to ID generator. Can be optionally specified at creation.
    IIdGenerator* idGenerator;

    /// Set of observers that will be notified when the update function will be called.
    std::vector<ISceneGraphUpdateObserver*> updateObservers;

    /// Policy on error propagation (e.g. duplicated IDs) to the observers. Default is true.
    bool callObserversEvenIfErrorsOccurred;


};

} // namespace brics_3d::rsg

} // namespace brics_3d
#endif

/* EOF */

