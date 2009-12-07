/**
 * @file 
 * IRigidTransformationEstimation.h
 *
 * @date: Dec 3, 2009
 * @author: sblume
 */

#ifndef IRIGIDTRANSFORMATIONESTIMATION_H_
#define IRIGIDTRANSFORMATIONESTIMATION_H_

#include "core/PointCloud3D.h"
#include "core/IHomogeneousMatrix44.h"

#include <vector>

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Abstract interface to estimate the rigid transformation between two corresponding point clouds.
 *
 * To estimate the transformation the mean square error function is minimized.
 */
class IRigidTransformationEstimation {
public:

	/**
	 * @brief Standard constructor
	 */
	IRigidTransformationEstimation();

	/**
	 * @brief Standard destructor
	 */
	virtual ~IRigidTransformationEstimation();

	/**
	 * @brief Estimates the rigid transformation between two corresponding point clouds.
	 *
	 * @param[in] pointCloud1 Pointer to first point cloud
	 * @param[in] pointCloud2 Pointer to second point cloud
	 * @param[in] pointPairs Pointer to point correspondences between points of pointCloud1 and pointCloud2
	 * @param[out] resultTransformation Pointer to resulting rigid transformation, represented as a homogeneous matrix
	 * @return error TODO
	 */
	virtual double estimateTransformation(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2, std::vector<CorrespondencePoint3DPair>* pointPairs, IHomogeneousMatrix44* resultTransformation) = 0;
};

}

#endif /* IRIGIDTRANSFORMATIONESTIMATION_H_ */

/* EOF */
