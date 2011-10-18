/**
 * @file 
 * INormalEstimation.h
 *
 * @date: Apr 26, 2011
 * @author: sblume
 */

#ifndef INORMALESTIMATION_H_
#define INORMALESTIMATION_H_

#include "core/PointCloud3D.h"
#include "core/NormalSet3D.h"

namespace BRICS_3D {

/**
 * @brief Abstract interface for normal estimation algorithms for point clouds
 */
class INormalEstimation {
public:
	INormalEstimation(){};
	virtual ~INormalEstimation(){};

	/**
	 * @brief Estimates normals in a point clouds.
	 *
	 * Each ith point in the cloud with have a corresponding ith normal.
	 *
	 * @param[in] pointCloud The input point cloud. This data will not
	 * be modified.
	 * @param[out] estimatedNormals Resulting set of estimated normals.
	 *
	 */
	void estimateNormals(PointCloud3D* pointCloud, NormalSet3D* estimatedNormals);
};

}

#endif /* INORMALESTIMATION_H_ */

/* EOF */
