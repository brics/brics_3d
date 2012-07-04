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

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "core/IHomogeneousMatrix44.h"
#include "Group.h"
#include "TimeStamp.h"
#include <vector>
using std::vector;
using std::pair;

namespace BRICS_3D {

namespace RSG {

typedef vector< pair<IHomogeneousMatrix44::IHomogeneousMatrix44Ptr, TimeStamp> >::iterator HistoryIterator;

/**
 * @brief Determine the accumulated transform along a path of nodes.
 * @param nodePath A path of nodes from root descending.
 * @return Shared pointer to the accumulated transform.
 * @ingroup sceneGraph
 */
extern IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getGlobalTransformAlongPath(Node::NodePath nodePath);

/**
 * @brief Calculate the accumulated global transform for a node.
 *
 * In case the node is a transform node it will be taken into account too.
 * In case the node has multiple paths to root node, the first found path will be taken!
 * @param node The node to where the transform from root will calculated.
 * @return Shared pointer to the accumulated transform.
 * @ingroup sceneGraph
 */
extern IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getGlobalTransform(Node::NodePtr node);

extern IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getTransformBetweenNodes(Node::NodePtr node, Node::NodePtr referenceNode);


/**
 * @brief A node that expresses a geometric transformation between its parents and children.
 * @ingroup sceneGraph
 *
 * The transform node can cache transform data over a certain period of time. Typically one
 * wants to get the most recent transform. This can be done with the getLatestTransform()
 * function. More advanced queries will have to invoke the getTransform(TimeStamp timeStamp)
 * function. See further details at the description of that function.
 */
class Transform : public Group {

  public:

	typedef boost::shared_ptr<RSG::Transform> TransformPtr;
	typedef boost::shared_ptr<RSG::Transform const> TransformConstPtr;

    Transform();

    virtual ~Transform();

    /**
     * @brief Add a new transform to the history.
     * @param newTransform The transform to be added.
     * @param timeStamp The times stamp that is associated with the transform.
     */
    void insertTransform(IHomogeneousMatrix44::IHomogeneousMatrix44Ptr newTransform, TimeStamp timeStamp);

    /**
     * @brief Retrieve a transform in the history cache whose time stamp matches best a given stamp.
     * @param timeStamp Time stamp to wich the temporal closest transform shall be found.
     * @return Shared pointer to the transform. It will be null in case that no transform is found.
     */
    IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getTransform(TimeStamp timeStamp);

    /**
     * @brief Retrieve the latest/newest transform.
     * @return Shared pointer to the transform. It will be null in case that no transform is found.
     */
    IHomogeneousMatrix44::IHomogeneousMatrix44Ptr getLatestTransform();

    /**
     * Getter for maxHistoryDuration
     */
    TimeStamp getMaxHistoryDuration();

    /**
     * Setter for maxHistoryDuration
     */
    void setMaxHistoryDuration(TimeStamp maxHistoryDuration);

    /**
     * @brief Returns the number of elements that are stored in the history cache.
     */
    unsigned int getCurrentHistoryLenght();

    /**
     * @brief Get the latest time stamp that is in the history cache.
     * @return The latest time stamp or 0.0 in case of an empty history cache.
     */
    TimeStamp getLatestTimeStamp();

    /**
     * @brief Get the oldest time stamp that is in the history cache.
     * @return The oldest time stamp or 0.0 in case of an empty history cache.
     */
    TimeStamp getOldestTimeStamp();

    /**
     * @brief Get the counter for how often this transform node has been updated.
     */
    unsigned int getUpdateCount();


    void accept(INodeVisitor* visitor);

    /**
     * @brief Delete all data from the history/cache that is older than the latestTimeStamp minus the duration of maxHistoryDuration.
     * @param latestTimeStamp The time stamp the defines the "current" time. To this stamp the maximim duration will relate.
     *        Plase note that in case that you do not pass the timeStamp from the history cache via getLatestTimeStamp() the complete history might
     *        get earased! Use the function outside the scope of this class with care.
     */
    void deleteOutdatedTransforms(TimeStamp latestTimeStamp);

  private:

    /**
     * @brief Determine the position in the history cache whose time stamp matches best a given stamp.
     * @param timeStamp Time stamp to wich the temporal closest data shall be found.
     * @return Iterator to the transform with closest time stamp. In case timeStamp is exactly in between
     * two times stamps of the cache the latest of both will be taken.
     */
    HistoryIterator getClosestTransform(TimeStamp timeStamp);

    /// History of transforms. Each transform has an associated time stamp.
    vector< pair<IHomogeneousMatrix44::IHomogeneousMatrix44Ptr, TimeStamp> > history;

    /// Iterator for the history data.
    HistoryIterator historyIterator;

    /// Maximum duration of storing the history of transforms.
    TimeStamp maxHistoryDuration; //TODO: should be of some Duration type not a time stamp...

    /// 10s in [ms] in case that the BRICS_3D::Timer is used, otherwise this number has no real meaning and should just serve as @p a default.
    static const long double dafaultMaxHistoryDuration = 10000.0;

    /// Counter for how often this transform node has been updated via insertTransform.
    unsigned int updateCount;
};

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

