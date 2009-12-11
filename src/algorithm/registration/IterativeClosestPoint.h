/**
 * @file 
 * IterativeClosestPoint.h
 *
 * @date: Dec 3, 2009
 * @author: sblume
 */

#ifndef ITERATIVECLOSESTPOINT_H_
#define ITERATIVECLOSESTPOINT_H_

#include "algorithm/registration/IIterativeClosestPoint.h"
#include "algorithm/registration/IPointCorrespondence.h"
#include "algorithm/registration/IRigidTransformationEstimation.h"

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Generic implementation of ICP
 *
 * This class serves a generic implementation of the Iterative Closest Point Algorithm.
 * It follows the "strategy" software design pattern (except that context and strategy are implemented in the same class).
 * That means the actual point correspondence and the rigid transformation estimation algorithms are exchangeable during runtime.
 */
class IterativeClosestPoint : public IIterativeClosestPoint {
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

	void match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation, IHomogeneousMatrix44* initalEstimate, int maxIterations = 20);

	double getConvergenceThreshold() const;

	int getMaxIterations() const;

	void setConvergenceThreshold(double convergenceThreshold);

	void setMaxIterations(int maxIterations);

	/**
	 * @brief Get the assigner algorithm that establishes point correspondences (e.g. k-d tree)
	 * @return Read-only pointer to assigner algorithm
	 */
    IPointCorrespondence* getAssigner() const;

	/**
	 * @brief Get the transformation estimation (error minimization) strategy
	 * @return Read-only pointer to transformation estimator
	 */
	IRigidTransformationEstimation* getEstimator() const;

	/**
	 * @brief Set the assigner algorithm that establishes point correspondences (e.g. k-d tree)
	 * @param[in] assigner Pointer to assigner algorithm
	 */
	void setAssigner(IPointCorrespondence* assigner);

	/**
	 * @brief Set the transformation estimation (error minimization) strategy
	 * @param[in] estimator Pointer to transformation estimator
	 */
	void setEstimator(IRigidTransformationEstimation* estimator);





private:

	///Pointer to point-to-point assigner strategy
	IPointCorrespondence* assigner;

	///Pointer to transformation estimation (error minimization) strategy
	IRigidTransformationEstimation* estimator;

	/// Defines the maximum amount of iterations for matching process
	int maxIterations;

	/// The threshold to define convergence.
	double convergenceThreshold;
};

}

#endif /* ITERATIVECLOSESTPOINT_H_ */

/* EOF */
