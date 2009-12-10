/**
 * @file 
 * RigidTransformationEstimationQUAT.h
 *
 * @date: Dec 10, 2009
 * @author: sblume
 */

#ifndef RIGIDTRANSFORMATIONESTIMATIONQUAT_H_
#define RIGIDTRANSFORMATIONESTIMATIONQUAT_H_

#include "IRigidTransformationEstimation.h"

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Implementation of rigid transformation estimation between two corresponding point clouds.
 *
 * This implementation uses a quaternion based error function for the ICP.
 */
class RigidTransformationEstimationQUAT: public BRICS_3D::IRigidTransformationEstimation {
public:

	/**
	 * Standard constructor
	 */
	RigidTransformationEstimationQUAT();

	/**
	 * Standard destructor
	 */
	virtual ~RigidTransformationEstimationQUAT();

	double estimateTransformation(std::vector<CorrespondencePoint3DPair>* pointPairs, IHomogeneousMatrix44* resultTransformation);

};

}

#endif /* RIGIDTRANSFORMATIONESTIMATIONQUAT_H_ */

/* EOF */
