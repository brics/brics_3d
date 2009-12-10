/**
 * @file 
 * RigidTransformationEstimationHELIX.h
 *
 * @date: Dec 10, 2009
 * @author: sblume
 */

#ifndef RIGIDTRANSFORMATIONESTIMATIONHELIX_H_
#define RIGIDTRANSFORMATIONESTIMATIONHELIX_H_

#include "IRigidTransformationEstimation.h"

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Implementation of rigid transformation estimation between two corresponding point clouds.
 *
 * This implementation uses a helix motion based error function for the ICP. Results might be inaccurate.
 */
class RigidTransformationEstimationHELIX: public BRICS_3D::IRigidTransformationEstimation {
public:

	/**
	 * Standard constructor
	 */
	RigidTransformationEstimationHELIX();

	/**
	 * Standard destructor
	 */
	virtual ~RigidTransformationEstimationHELIX();

	double estimateTransformation(std::vector<CorrespondencePoint3DPair>* pointPairs, IHomogeneousMatrix44* resultTransformation);

};

}

#endif /* RIGIDTRANSFORMATIONESTIMATIONHELIX_H_ */

/* EOF */
