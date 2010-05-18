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
 * @ingroup registration
 * @brief Abstract stateless interface for the Iterative Closest Point (ICP) registration algorithm
 *
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
	 *
	 * <b>NOTE:</b> To apply an initial transformation ("initial guess") to one of the point clouds: invoke the homogeneous
	 * transformation directly on the point cloud data-type before starting the ICP registration. See also Point3D::homogeneousTransformation()
	 */
	virtual void match(PointCloud3D* model, PointCloud3D* data, IHomogeneousMatrix44* resultTransformation) = 0;

};

}

#endif /* IITERATIVECLOSESTPOINT_H_ */

/* EOF */
