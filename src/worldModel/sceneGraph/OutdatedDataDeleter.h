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

#ifndef OUTDATEDDATADELETER_H_
#define OUTDATEDDATADELETER_H_

#include "Node.h"
#include "Group.h"
#include "Transform.h"
#include "GeometricNode.h"
#include "INodeVisitor.h"

#include "util/Timer.h"

namespace BRICS_3D {

namespace RSG {

/**
 * Visitor that traverses the graph and deletes transform nodes that are outdated.
 */
class OutdatedDataDeleter : public INodeVisitor {
public:
	OutdatedDataDeleter();
	virtual ~OutdatedDataDeleter();

	virtual void visit(Node* node);
	virtual void visit(Group* node);
	virtual void visit(Transform* node);
	virtual void visit(GeometricNode* node);

    bool getPerformAutomaticHistoryUpdates() const;
    void setPerformAutomaticHistoryUpdates(bool performAutomaticHistoryUpdates);

    unsigned int getMinHistoryLength() const;
    void setMinHistoryLength(unsigned int minHistoryLength);

private:

    /// Timer to be able to campare history with current time
	Timer timer; //TODO unfortunately the Timer is in brics_3d_util...

	/**
	 * Flag to toggle on automatic updates of the transform caches.
	 * This essentially means to invoke BRICS_3D::RSG::Transform::deleteOutdatedTransforms() for each visited transform.
	 *
	 * The default is true.
	 */
	bool performAutomaticHistoryUpdates;

	/**
	 * Threshold for deletion policy.
	 * If history/cache length gets smaller than this value the corresponding transform node
	 * will be deleted.
	 *
	 * The default is 1.
	 *
	 * @note Remember, there should be at least a chance to fill/build up the history cache.
	 * I.e. if you delete more often than insert updates everything will be deleted...
	 * However this is out of the scope of this class.
	 */
	unsigned int minHistoryLength;

};

}

}

#endif /* OUTDATEDDATADELETER_H_ */

/* EOF */
