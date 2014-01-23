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

#ifndef BRICS_3D_WORLDMODEL_H
#define BRICS_3D_WORLDMODEL_H

#include "IWorldModelQuery.h"
#include "IWorldModelUpdate.h"
#include "IWorldModelCoordination.h"
#include "sceneGraph/SceneGraphFacade.h"
#include "sceneGraph/OutdatedDataDeleter.h"
#include "sceneGraph/OutdatedDataIdAwareDeleter.h"
#include "sceneGraph/TimeStamp.h"
#include "brics_3d/util/Timer.h"

struct ubx_node_info; // forward declaration to hide microblx internals from wm users

namespace brics_3d {

/**
 * @brief <b>The</b> handle for a 3D world model.
 * @ingroup sceneGraph
 */
class WorldModel : public IWorldModelQuery, public IWorldModelUpdate, public IWorldModelCoordination {

  public:
    WorldModel();

    WorldModel(rsg::IIdGenerator* idGenerator);

    virtual ~WorldModel();

    /* Implemented interfaces: */

    void getSceneObjects(vector<rsg::Attribute> attributes, vector<SceneObject>& results);

    void getCurrentTransform(rsg::Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform);

    void insertTransform(rsg::Id id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform);

    void addSceneObject(SceneObject newObject, rsg::Id& assignedId);

    void initPerception();

    void runPerception();

    void runOncePerception();

    void stopPerception();

    /* Function block interface */
    bool loadFunctionBlock(std::string name);
    bool executeFunctionBlock(std::string name, std::vector<rsg::Id>& input, std::vector<rsg::Id>& output);
    bool getloadedFunctionBlocks(std::vector<std::string>& functionBlocks);


    /* Helper functions */
    rsg::Id getRootNodeId();

    brics_3d::rsg::TimeStamp now();

    brics_3d::rsg::SceneGraphFacade scene;

  private:

    Timer timer; //TODO unfortunately here we introduce a dependency to the brics_3d_util lib...

    /// Root path to function blocks
    std::string functionBlockPath;

#ifdef BRICS_MICROBLX_ENABLE
    /// THE handle that handles all function blocks
    ubx_node_info* microBlxNodeHandle;
  public:

    static WorldModel* microBlxWmHandle; //workaround for wm as a shared resource among the microblx
#endif

};

} // namespace brics_3d
#endif

/* EOF */

