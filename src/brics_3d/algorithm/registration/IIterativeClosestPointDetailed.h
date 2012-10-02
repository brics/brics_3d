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

#ifndef BRICS_3D_IITERATIVECLOSESTPOINTDETAILED_H_
#define BRICS_3D_IITERATIVECLOSESTPOINTDETAILED_H_

#include "brics_3d/core/IHomogeneousMatrix44.h"

namespace brics_3d {

/**
 * @ingroup registration
 * @brief Abstract stateful interface for ICP
 *
 * This is a stateful interface, that relies on previous states e.g. invocations of setData(), setModel() or
 * performNextIteration().
 * To correctly use this interface first set data and model, then invoke performNextIteration
 * as long as you want (probably until the error is smaller than a certain value).
 */
class IIterativeClosestPointDetailed {
public:

	/**
	 * Standard constructor
	 */
	IIterativeClosestPointDetailed(){};

	/**
	 * Standard destructor
	 */
	virtual ~IIterativeClosestPointDetailed(){};

	/**
	 * @brief Set the "data"
	 * @param data Pointer to data point cloud
	 */
	virtual void setData(PointCloud3D* data) = 0;

	/**
	 * @brief Set the "model"
	 * @param model Pointer to data point cloud
	 */
	virtual void setModel(PointCloud3D* model) = 0;

	/**
	 * @brief Get data
	 * @return Pointer to data point cloud, 0 if data was not set before
	 */
	virtual PointCloud3D* getData() = 0;

	/**
	 * @brief Get model
	 * @return Pointer to data point cloud, 0 if data was not set before
	 */
	virtual PointCloud3D* getModel() = 0;

	/**
	 * @brief Perform one step/iteration of the ICP algorithm
	 *
	 * This is a stateful function, that relies on previous states. Thus the output is dependent on
	 * what kind of data was set in setData() and setModel() and how many time performNextIteration()
	 * has been invoked before. <br>
	 * <br>Assumption:<br> model and data are set before! <br>
	 * Note that the data will be updated during this operation.
	 *
	 * <b>NOTE:</b> To apply an initial transformation ("initial guess") to one of the point clouds: invoke the homogeneous
	 * transformation directly on the point cloud data-type before starting the ICP registration. See also Point3D::homogeneousTransformation()
	 */
	virtual double performNextIteration() = 0;

	/**
	 * @brief Get the estimated rigid transformation in the last ICP iteration.
	 * @return Pointer to transformation, 0 if no ICP iteration was done before
	 */
	virtual IHomogeneousMatrix44* getLastEstimatedTransformation() = 0;

	/**
	 * @brief Get the accumulated rigid transformation in the last ICP iteration.
	 * If no more iterations are performed this will be the final result.
	 *
	 * @return Pointer to transformation, 0 if no ICP iteration was done before
	 */
	virtual IHomogeneousMatrix44*  getAccumulatedTransfomation() = 0;



};

}

#endif /* BRICS_3D_IITERATIVECLOSESTPOINTDETAILED_H_ */

/* EOF */
