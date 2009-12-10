/**
 * @file 
 * RigidTransformationEstimationAPX.h
 *
 * @date: Dec 10, 2009
 * @author: sblume
 */

#ifndef RIGIDTRANSFORMATIONESTIMATIONAPX_H_
#define RIGIDTRANSFORMATIONESTIMATIONAPX_H_

#include "IRigidTransformationEstimation.h"

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Implementation of rigid transformation estimation between two corresponding point clouds.
 *
 * This implementation uses a APX TODO based error function for the ICP.
 */
class RigidTransformationEstimationAPX: public BRICS_3D::IRigidTransformationEstimation {
public:

	/**
	 * Standard constructor
	 */
	RigidTransformationEstimationAPX();

	/**
	 * Standard destructor
	 */
	virtual ~RigidTransformationEstimationAPX();

	double estimateTransformation(std::vector<CorrespondencePoint3DPair>* pointPairs, IHomogeneousMatrix44* resultTransformation);

};

}

#endif /* RIGIDTRANSFORMATIONESTIMATIONAPX_H_ */

/* EOF */
