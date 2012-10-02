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

#ifndef ITERATIVECLOSESTPOINT_H_
#define ITERATIVECLOSESTPOINT_H_

#include "core/Logger.h"
#include "algorithm/registration/IIterativeClosestPoint.h"
#include "algorithm/registration/IIterativeClosestPointSetup.h"
#include "algorithm/registration/IIterativeClosestPointDetailed.h"
#include "algorithm/registration/IPointCorrespondence.h"
#include "algorithm/registration/IRigidTransformationEstimation.h"

namespace brics_3d {

/**
 * @ingroup registration
 * @brief Generic implementation of ICP
 *
 * This class serves a generic implementation of the Iterative Closest Point Algorithm.
 * It follows the "strategy" software design pattern (except that context and strategy are implemented in the same class).
 * That means the actual point correspondence and the rigid transformation estimation algorithms are exchangeable during runtime.
 */
class IterativeClosestPoint : public IIterativeClosestPoint, public IIterativeClosestPointSetup, public IIterativeClosestPointDetailed {
public:

	/**
	 * @brief Standard constructor
	 */
	IterativeClosestPoint();

	/**
	 * @brief Constructor that allows to fully setup the ICP
	 */
	IterativeClosestPoint(IPointCorrespondence *assigner, IRigidTransformationEstimation *estimator, double convergenceThreshold = 0.00001, int maxIterations = 20);

	/**
	 * @brief Standard destructor
	 */
	virtual ~IterativeClosestPoint();

	void match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation);

	double getConvergenceThreshold() const;

	int getMaxIterations() const;

	void setConvergenceThreshold(double convergenceThreshold);

	void setMaxIterations(int maxIterations);

	void setAssigner(IPointCorrespondence* assigner);

	void setEstimator(IRigidTransformationEstimation* estimator);

    IPointCorrespondence* getAssigner() const;

	IRigidTransformationEstimation* getEstimator() const;

	void setData(PointCloud3D* data);

	void setModel(PointCloud3D* model);

	PointCloud3D* getData();

	PointCloud3D* getModel();

	double performNextIteration();

	IHomogeneousMatrix44*  getLastEstimatedTransformation();

	IHomogeneousMatrix44*  getAccumulatedTransfomation();



private:

	///Pointer to point-to-point assigner strategy
	IPointCorrespondence* assigner;

	///Pointer to transformation estimation (error minimization) strategy
	IRigidTransformationEstimation* estimator;

	/// Defines the maximum amount of iterations for matching process
	int maxIterations;

	/// The threshold to define convergence.
	double convergenceThreshold;

	///Pointer to the model for the stateful interface (IIterativeClosestPointDetailed)
	PointCloud3D* model;

	///Pointer to the model for the stateful interface (IIterativeClosestPointDetailed)
	PointCloud3D* data;

	///Pointer to the model for the stateful interface (IIterativeClosestPointDetailed)
	IHomogeneousMatrix44* intermadiateTransformation;

	///Pointer to the model for the stateful interface (IIterativeClosestPointDetailed)
	IHomogeneousMatrix44* resultTransformation;

public:

	double icpResultError; //FIXME move to getter methods in IIterativeClosestPoint
	int icpresultIterations;

};

}

#endif /* ITERATIVECLOSESTPOINT_H_ */

/* EOF */
