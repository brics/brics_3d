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
 */
class IterativeClosestPoint : public IIterativeClosestPoint {
public:

	/**
	 * @brief Standard constructor
	 */
	IterativeClosestPoint();

	/**
	 * @brief Standard destructor
	 */
	virtual ~IterativeClosestPoint();

	void match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation, int maxIterations = 20);

	void match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation, IHomogeneousMatrix44* initalEstimate, int maxIterations = 20);

private:

	///Pointer to point-to-point assinger strategy
	IPointCorrespondence* assigner;

	///Pointer to transformation estimation (error minimization) strategy
	IRigidTransformationEstimation* estimator;


};

}

#endif /* ITERATIVECLOSESTPOINT_H_ */

/* EOF */
