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

#ifndef RSG_OUTDATEDDATAIDAWAREDELETER_H_
#define RSG_OUTDATEDDATAIDAWAREDELETER_H_

#include "OutdatedDataDeleter.h"
#include "SceneGraphFacade.h"

namespace brics_3d {

namespace rsg {

/**
 * A more specialized visitor version to be used in conjunction with a SceneGraphFacade.
 *
 * This class behaves essentially the same as OutdatedDataDeleter, except that it will
 * make use of the deleteNode function in the facade. Thus the IDs managed by the facade
 * will be in sync.
 *
 * @ingroup sceneGraph
 */
class OutdatedDataIdAwareDeleter : public OutdatedDataDeleter {
public:
	OutdatedDataIdAwareDeleter(SceneGraphFacade* facadeHandle);
	virtual ~OutdatedDataIdAwareDeleter();

	void doDeleteNode(Node* node);

private:
	SceneGraphFacade* facadeHandle;
};

}

}

#endif /* RSG_OUTDATEDDATAIDAWAREDELETER_H_ */

/* EOF */
