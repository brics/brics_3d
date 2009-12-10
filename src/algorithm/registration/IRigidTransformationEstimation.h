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
#include "core/CorrespondencePoint3DPair.h"

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
	IRigidTransformationEstimation(){};

	/**
	 * @brief Standard destructor
	 */
	virtual ~IRigidTransformationEstimation(){};

	/**
	 * @brief Estimates the rigid transformation between two corresponding point clouds.
	 *
	 * @param[in] pointPairs Pointer to point correspondences between points of pointCloud1 and pointCloud2
	 * @param[out] resultTransformation Pointer to resulting rigid transformation, represented as a homogeneous matrix
	 * @return Returns the RMS point-to-point error
	 *
	 * NOTE: A function with two point clouds a correspondence representation based on CorrespondenceIndexPair might be more efficient,
	 * as the pointPairs vector does not need to be filled, but this implies that however the correspondences are established, the underlying structure is
	 * based on a vector. This assumption does not always hold e.g. the k-d tree does not necessarily store its data in a consecutive vector.
	 */
	virtual double estimateTransformation(std::vector<CorrespondencePoint3DPair>* pointPairs, IHomogeneousMatrix44* resultTransformation) = 0;

};

}

#endif /* IRIGIDTRANSFORMATIONESTIMATION_H_ */

/* EOF */
