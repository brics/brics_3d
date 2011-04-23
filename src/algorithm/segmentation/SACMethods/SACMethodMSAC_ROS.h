/*
 * SACMethodMSAC_ROS.h
 *
 *  Created on: Apr 21, 2011
 *      Author: reon
 */

#ifndef SACMETHODMSAC_ROS_H_
#define SACMETHODMSAC_ROS_H_

#include "ISACMethods.h"


namespace BRICS_3D {

class SACMethodMSAC_ROS : public ISACMethods{

public:

	SACMethodMSAC_ROS();
	bool computeModel();
	virtual ~SACMethodMSAC_ROS();
};

}

#endif /* SACMETHODMSAC_ROS_H_ */
