/*
 * SACMethodRANSAC_ROS.h
 *
 *  Created on: Apr 19, 2011
 *      Author: reon
 */

#ifndef SACMETHODRANSAC_ROS_H_
#define SACMETHODRANSAC_ROS_H_

#include "ISACMethods.h"


namespace BRICS_3D {

class SACMethodRANSAC_ROS : public ISACMethods {

public:

	SACMethodRANSAC_ROS();
	bool computeModel ();
	virtual ~SACMethodRANSAC_ROS();
};

}

#endif /* SACMETHODRANSAC_ROS_H_ */
