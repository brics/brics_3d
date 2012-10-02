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

#ifndef BRICS_3D_IREGISTRATION_H_
#define BRICS_3D_IREGISTRATION_H_

#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/IHomogeneousMatrix44.h"

namespace brics_3d {

/**
 * @brief Generic interface for a point cloud registration componennt.
 * @ingroup registration
 *
 */
class IRegistration {
public:
	IRegistration(){};
	virtual ~IRegistration(){};

	/**
	 * @brief Calculates the required transformation to align the data point cloud to the model point cloud
	 *
	 * @param[in] model Pointer to point cloud that represents the model
	 * @param[in,out] data Pointer to point cloud that represents the data (points will be transformed during iterations)
	 * @param[out] resultTransformation Pointer to resulting homogeneous transformation to align both point clouds
	 */
	virtual void match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation) = 0;

};

}

#endif /* BRICS_3D_IREGISTRATION_H_ */

/* EOF */
