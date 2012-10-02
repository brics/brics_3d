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

#ifndef IITERATIVECLOSESTPOINTSETUP_H_
#define IITERATIVECLOSESTPOINTSETUP_H_

#include "algorithm/registration/IPointCorrespondence.h"
#include "algorithm/registration/IRigidTransformationEstimation.h"

namespace brics_3d {

/**
 * @ingroup registration
 * @brief Abstract interface to setup and configure the ICP
 */
class IIterativeClosestPointSetup {
public:

	/**
	 * @brief Standard constructor
	 */
	IIterativeClosestPointSetup(){};

	/**
	 * @brief Standard destructor
	 */
	virtual ~IIterativeClosestPointSetup(){};

	/**
	 * @brief Get the threshold to define convergence
	 * @return The convergence threshold
	 */
	virtual double getConvergenceThreshold() const = 0;

	/**
	 * @brief Get the maximum amount of iterations for matching process
	 * @return The maximum amount of iterations
	 */
	virtual int getMaxIterations() const = 0;

	/**
	 * @brief Set the threshold to define convergence
	 * @param convergenceThreshold The convergence threshold
	 */
	virtual void setConvergenceThreshold(double convergenceThreshold) = 0;

	/**
	 * @brief Set the maximum amount of iterations for matching process
	 * @param maxIterations The maximum amount of iterations
	 */
	virtual void setMaxIterations(int maxIterations) = 0;

	/**
	 * @brief Set the assigner algorithm that establishes point correspondences (e.g. k-d tree)
	 * @param[in] assigner Pointer to assigner algorithm
	 */
	virtual void setAssigner(IPointCorrespondence* assigner) = 0;

	/**
	 * @brief Set the transformation estimation (error minimization) strategy
	 * @param[in] estimator Pointer to transformation estimator
	 */
	virtual void setEstimator(IRigidTransformationEstimation* estimator) = 0;

	/**
	 * @brief Get the assigner algorithm that establishes point correspondences (e.g. k-d tree)
	 * @return Read-only pointer to assigner algorithm
	 */
	virtual IPointCorrespondence* getAssigner() const = 0;

	/**
	 * @brief Get the transformation estimation (error minimization) strategy
	 * @return Read-only pointer to transformation estimator
	 */
	virtual IRigidTransformationEstimation* getEstimator() const = 0;
};

}

#endif /* IITERATIVECLOSESTPOINTSETUP_H_ */

/* EOF */
