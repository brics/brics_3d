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

#ifndef RSG_ISCENEGRAPHERROROBSERVER_H_
#define RSG_ISCENEGRAPHERROROBSERVER_H_


namespace brics_3d {

namespace rsg {

/**
 * @brief Interface to observe potential errors of the scenegraph
 * @ingroup sceneGraph
 */
class ISceneGraphErrorObserver  {

public:

	enum SceneGraphErrorCode {
		RSG_ERR_NO_ERROR = 0,
		RSG_ERR_ID_EXISTS_ALREADY = 1,  			// Warning. Cf. "Forced ID ... cannot be assigned. Probably another object with that ID exists already!";
		RSG_ERR_ID_DOES_NOT_EXIST = 2,			// Error. The node is missing - indicates data loss
		RSG_ERR_PARENT_ID_DOES_NOT_EXIST = 3,	// Error. The parent node is missing - indicates data loss, or wrong order of traversal in case of initial creation
		RSG_ERR_UPDATE_IS_NOT_NEWER = 4, 			// Warning. Ensures temporal order of updates (for Attributes)
		RSG_ERR_UPDATE_IS_IDENTICAL = 5,			// Warning. Prevents ping pong of same attribute updates
		RSG_ERR_GRAPH_CYCLE_DETECTED = 6,			// Error. Prevents insertion of cycles in a DAG
		RSG_ERR_NODE_IS_OF_DIFFERENT_TYPE = 7   // Error. The caller expect a different type
	};

	virtual void onError(SceneGraphErrorCode code) = 0;

};

}

}

#endif /* RSG_ISCENEGRAPHUPDATEOBSERVER_H_ */

/* EOF */
