/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2016, KU Leuven
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

#ifndef RSG_SEMANTICCONTEXTUPDATEFILTER_H_
#define RSG_SEMANTICCONTEXTUPDATEFILTER_H_

#include "brics_3d/worldModel/sceneGraph/ISceneGraphUpdateObserver.h"
#include "brics_3d/worldModel/WorldModel.h"
#include "brics_3d/worldModel/sceneGraph/Attribute.h"
#include "brics_3d/worldModel/sceneGraph/GraphConstraint.h"
#include "brics_3d/worldModel/sceneGraph/LODCalculator.h"


namespace brics_3d {
namespace rsg {


/**
 * @brief Filters updated based on graph constraints.
 */
class GraphConstraintUpdateFilter :  public ISceneGraphUpdateObserver {
public:

	/// Important to correctly interpret send/receive actions in graph constraints.
	enum UpdateMode {
		UNDEFINDE_MODE = 0,
		SENDER = 1,
		RECEIVER = 2
	};

	GraphConstraintUpdateFilter(WorldModel* wm, UpdateMode mode = SENDER);
	virtual ~GraphConstraintUpdateFilter();

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

	const string& getNameSpaceIdentifier() const;

	void setNameSpaceIdentifier(const string& nameSpaceIdentifier);

    /// THE local graph constraints, as specified by this root node
    std::vector<GraphConstraint> constraints;

    /// THE remote graph constraints, as specified by other root nodes

    /// Implements the constraint checking algorithm
    bool checkConstraint(GraphConstraint constraint, GraphConstraint::Type type, double frequencyInHz, double distanceInMeters, double lod, Id assignedId, vector<Attribute> attributes);

    /// Implements the constraint checking part involving a comparison
    bool checkComparision(GraphConstraint constraint, GraphConstraint::Type type, double value, double allowedValue, string tag);

    /// Implements extraction of constraints from an attribute set.
    bool getConstraintsFromAttributes(vector<Attribute> attributes, std::vector<GraphConstraint>& constraints);

private:

    /// input or Output
    UpdateMode mode;

    /// Set of observers that will be notified when the update function will be called and the Semantic Context constraints are satisfied.
    std::vector<ISceneGraphUpdateObserver*> updateObservers;

    /// A SemnticContext is identified by its name space prefix.
    string nameSpaceIdentifier;

    /// Internal query used to check if at least one attribute has belongs to a Semantic Context
    string query;

    /// Handle for internal queries; It has to be the WordModel rather than the SceneGraphFacade since we need time here.
    WorldModel* wm;

    /// Per type time stamps. Relevant for FREQUENCY constraints.
    TimeStamp lastSendType[GraphConstraint::TYPE_NR_ITEMS];

    /// Get LOD from Shapes in case of LOD constraints
    LODCalculator lodCalculator;

    const static std::string contraintKey;
};


} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_SEMANTICCONTEXTUPDATEFILTER_H_ */

/* EOF */
