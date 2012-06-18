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

#ifndef ISCENEGRAPHUPDATEOBSERVER_H_
#define ISCENEGRAPHUPDATEOBSERVER_H_

#include "ISceneGraphUpdate.h"

namespace BRICS_3D {

namespace RSG {

/**
 * @brief Interface to observe any updates to the scenegraph
 * @ingroup sceneGraph
 */
class ISceneGraphUpdateObserver : public ISceneGraphUpdate {


};

}

}

#endif /* ISCENEGRAPHUPDATEOBSERVER_H_ */

/* EOF */
