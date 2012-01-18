/*
 * @file: SACMethodMSAC.h
 *
 * @date: Apr 21, 2011
 * @author: reon
 */

#ifndef SACMETHODMSAC_H_
#define SACMETHODMSAC_H_

#include "ISACMethods.h"


namespace BRICS_3D {

/**
 * @brief class implementing the MSAC method.
 * @ingroup segmentation
 *
 * The code is a re-factored version of the algorithm implementation in ROS:PCL
 */
//ToDo add reference to the method
class SACMethodMSAC : public ISACMethods{

public:

	SACMethodMSAC();
	bool computeModel();
	virtual ~SACMethodMSAC();
};

}

#endif /* SACMETHODMSAC_H_ */
