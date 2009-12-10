/**
 * @file 
 * RigidTransformationEstimationORTHO.h
 *
 * @date: Dec 10, 2009
 * @author: sblume
 */

#ifndef RIGIDTRANSFORMATIONESTIMATIONORTHO_H_
#define RIGIDTRANSFORMATIONESTIMATIONORTHO_H_

#include "IRigidTransformationEstimation.h"

namespace BRICS_3D {

/**
 * @ingroup registration
 * @brief Implementation of rigid transformation estimation between two corresponding point clouds.
 *
 * This implementation uses a ORTHO TODO based error function for the ICP.
 */
class RigidTransformationEstimationORTHO: public BRICS_3D::IRigidTransformationEstimation {
public:

	/**
	 * Standard constructor
	 */
	RigidTransformationEstimationORTHO();

	/**
	 * Standard destructor
	 */
	virtual ~RigidTransformationEstimationORTHO();

	double estimateTransformation(std::vector<CorrespondencePoint3DPair>* pointPairs, IHomogeneousMatrix44* resultTransformation);

};

}

#endif /* RIGIDTRANSFORMATIONESTIMATIONORTHO_H_ */

/* EOF */
