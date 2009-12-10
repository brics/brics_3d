/**
 * @file 
 * IIterativeClosestPoint.h
 *
 * @date: Nov 24, 2009
 * @author: sblume
 */

#ifndef IITERATIVECLOSESTPOINT_H_
#define IITERATIVECLOSESTPOINT_H_

#include "core/PointCloud3D.h"
#include "core/IHomogeneousMatrix44.h"

namespace BRICS_3D {

/**
 * @defgroup registration Registration
 * @brief This module contains algorithms for registration of point clouds.
 *
 * The module comprises in particular the Iterative Closest Point algorithm (ICP) and atomic sub components.
 *
 */

/**
 * @ingroup registration
 * @brief Abstract interface for the Iterative Closest Point (ICP) registration algorithm
 */
class IIterativeClosestPoint {
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
	 */
	virtual void match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation) = 0;

	/**
	 * @brief Calculates the required transformation to align the data point cloud to the model point cloud
	 *
	 * @param[in] model Pointer to point cloud that represents the model
	 * @param[in] data Pointer to point cloud that represents the data
	 * @param[out] resultTransformation Pointer to resulting homogeneous transformation to align both point clouds
	 * @param[in] initalEstimate Pointer to homogeneous matrix that represents an initial estimate of the transformation to align both point clouds
	 * @param maxIterations Maximum amount of iterations for matching process
	 */
	//virtual void match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation, IHomogeneousMatrix44* initalEstimate, int maxIterations = 20) = 0;

};

}

#endif /* IITERATIVECLOSESTPOINT_H_ */

/* EOF */
