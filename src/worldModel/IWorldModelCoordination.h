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

#ifndef BRICS_3D_IWORLDMODELCOORDINATION_H
#define BRICS_3D_IWORLDMODELCOORDINATION_H

namespace brics_3d {

/**
 * @brief Coordination interface to trigger the internal perception processing functionalities.
 * @ingroup sceneGraph
 */
class IWorldModelCoordination {
  public:
    virtual void initPerception() = 0;

    virtual void runPerception() = 0;

    virtual void runOncePerception() = 0;

    virtual void stopPerception() = 0;

};

} // namespace brics_3d
#endif

/* EOF */

