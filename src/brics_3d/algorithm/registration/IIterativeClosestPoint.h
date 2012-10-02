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

#ifndef BRICS_3D_IITERATIVECLOSESTPOINT_H_
#define BRICS_3D_IITERATIVECLOSESTPOINT_H_

#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/IHomogeneousMatrix44.h"
#include "IRegistration.h"

namespace brics_3d {

/**
 * @ingroup registration
 * @brief Abstract stateless interface for the Iterative Closest Point (ICP) registration algorithm
 *
 */
class IIterativeClosestPoint : public IRegistration {
public:

	/**
	 * @brief Standard constructor
	 */
	IIterativeClosestPoint(){};

	/**
	 * @brief Standard destructor
	 */
	virtual ~IIterativeClosestPoint(){};

	/**
	 * @brief Calculates the required transformation to align the data point cloud to the model point cloud
	 *
	 * @param[in] model Pointer to point cloud that represents the model
	 * @param[in,out] data Pointer to point cloud that represents the data (points will be transformed during iterations)
	 * @param[out] resultTransformation Pointer to resulting homogeneous transformation to align both point clouds
	 *
	 * <b>NOTE:</b> To apply an initial transformation ("initial guess") to one of the point clouds: invoke the homogeneous
	 * transformation directly on the point cloud data-type before starting the ICP registration. See also Point3D::homogeneousTransformation()
	 */
	virtual void match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation) = 0;

};

}

#endif /* BRICS_3D_IITERATIVECLOSESTPOINT_H_ */

/* EOF */
