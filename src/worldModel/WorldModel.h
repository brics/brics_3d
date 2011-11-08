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

#ifndef WORLDMODEL_H
#define WORLDMODEL_H

#include "IWorldModelQuery.h"
#include "sceneGraph/SceneManager.h"
#include "IWorldModelUpdate.h"
#include "IWorldModelCoordination.h"

namespace BRICS_3D {

/**
 * @brief <b>The</b> handle for a 3D world model.
 */
class WorldModel : public IWorldModelQuery, public IWorldModelUpdate, public IWorldModelCoordination {

  public:
    WorldModel();

    virtual ~WorldModel();

    /* Implemented interfaces: */

    void getSceneObjects(vector<Attribute> attributes, vector<SceneObject>& results);

    void getCurrentTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform);

    void insertTransform(unsigned int id, IHomogeneousMatrix44::IHomogeneousMatrix44Ptr transform);

    void addSceneObject(SceneObject newObject, unsigned int& assignedId);

    void initPerception();

    void runPerception();

    void runOncePerception();

    void stopPerception();

    /* Helper functions */
    unsigned int getRootNodeId();

    BRICS_3D::RSG::SceneManager scene;
};

} // namespace BRICS_3D
#endif

/* EOF */

