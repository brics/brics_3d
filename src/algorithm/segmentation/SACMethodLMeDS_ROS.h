/*
 * SACMethodLMeDS_ROS.h
 *
 *  Created on: Apr 21, 2011
 *      Author: reon
 */

#ifndef SACMETHODLMEDS_H_
#define SACMETHODLMEDS_H_
#include "ISACMethods.h"
#include "IObjectModel.h"

namespace BRICS_3D {

class SACMethodLMeDS_ROS : public ISACMethods{
public:
	SACMethodLMeDS_ROS();
	bool computeModel();
	virtual ~SACMethodLMeDS_ROS();
};

}

#endif /* SACMETHODLMEDS_H_ */
