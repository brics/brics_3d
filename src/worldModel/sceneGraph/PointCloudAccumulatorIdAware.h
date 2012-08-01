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


#ifndef POINTCLOUDACCUMULATORIDAWARE_H_
#define POINTCLOUDACCUMULATORIDAWARE_H_

#include "PointCloudAccumulator.h"
#include "SceneGraphFacade.h"

namespace BRICS_3D {

namespace RSG {

/**
 * @brief A more specialized visitor version of BRICS_3D::RSG::PointCloudAccumulator to be used in conjunction with a SceneGraphFacade.
 *
 * The visitor is essentially the same as BRICS_3D::RSG::PointCloudAccumulator except that it uses an ID do define the referene node.
 *
 * @ingroup sceneGraph
 */
class PointCloudAccumulatorIdAware : public PointCloudAccumulator {
public:

	/**
	 * @brief Constructor with reference node defined be ID and not by (shared) pointer
	 * @param facadeHandle Hande to the scene graph facade as it manages the IDs.
	 * @param referenceNodeId The Cartesian frame that is valid for the node will be used as reference to interpret the 3D points.
	 */
	PointCloudAccumulatorIdAware(SceneGraphFacade* facadeHandle, unsigned int referenceNodeId);

	/**
	 * @brief Default destructor.
	 */
	virtual ~PointCloudAccumulatorIdAware();

	/**
	 * @brief Overridden template method that uses IDs interanlly instead of (shared) pointers.
	 */
	virtual IHomogeneousMatrix44::IHomogeneousMatrix44Ptr doGetTransformFromReferenceToPointCloud(GeometricNode* node);

private:
	SceneGraphFacade* facadeHandle;
	unsigned int referenceNodeId;
};

}

}

#endif /* POINTCLOUDACCUMULATORIDAWARE_H_ */

/* EOF */
