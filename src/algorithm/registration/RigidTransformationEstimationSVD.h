/**
 * @file 
 * RigidTransformationEstimationSVD.h
 *
 * @date: Dec 7, 2009
 * @author: sblume
 */

#ifndef RIGIDTRANSFORMATIONESTIMATIONSVD_H_
#define RIGIDTRANSFORMATIONESTIMATIONSVD_H_

#include "IRigidTransformationEstimation.h"

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Implementation of rigid transformation estimation between two corresponding point clouds.
 *
 * This implementation bases on a singular value decomposition (SVD) for the ICP error function.
 */
class RigidTransformationEstimationSVD: public BRICS_3D::IRigidTransformationEstimation {
public:

	/**
	 * Standard constructor
	 */
	RigidTransformationEstimationSVD();

	/**
	 * Standard destructor
	 */
	virtual ~RigidTransformationEstimationSVD();

	double estimateTransformation(PointCloud3D* pointCloud1, PointCloud3D* pointCloud2, std::vector<CorrespondencePoint3DPair>* pointPairs, IHomogeneousMatrix44* resultTransformation);
};

}

#endif /* RIGIDTRANSFORMATIONESTIMATIONSVD_H_ */

/* EOF */
