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

namespace brics_3d {

namespace rsg {

/**
 * Visitor that traverses the graph and deletes transform nodes that are outdated.
 *
 * This class also serves as a "Template method pattern" whereas the node deletion
 * implementation might vary but the skeleton of the deletion algorithm will be
 * preserved. Thus the doDeleteNode is the one and only one primitive operation that
 * might be overriden by another conrete implementation.
 *
 * @ingroup sceneGraph
 */
class OutdatedDataDeleter : public INodeVisitor {
public:
	OutdatedDataDeleter();
	virtual ~OutdatedDataDeleter();

	void visit(Node* node);
	void visit(Group* node);
	void visit(Transform* node);
	void visit(GeometricNode* node);

    bool getPerformAutomaticHistoryUpdates() const;
    void setPerformAutomaticHistoryUpdates(bool performAutomaticHistoryUpdates);

    unsigned int getMinHistoryLength() const;
    void setMinHistoryLength(unsigned int minHistoryLength);

    virtual void doDeleteNode(Node* node);

private:

    /// Timer to be able to campare history with current time
	Timer timer; //TODO unfortunately the Timer is in brics_3d_util...

	/**
	 * Flag to toggle on automatic updates of the transform caches.
	 * This essentially means to invoke brics_3d::rsg::Transform::deleteOutdatedTransforms() for each visited transform.
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
