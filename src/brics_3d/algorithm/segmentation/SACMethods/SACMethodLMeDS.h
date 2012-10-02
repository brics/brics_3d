/*
 * @file: SACMethodLMeDS_ROS.h
 *
 * @date: Apr 21, 2011
 * @author: reon
 */

#ifndef SACMETHODLMEDS_H_
#define SACMETHODLMEDS_H_

#include "ISACMethods.h"

namespace brics_3d {

/**
 * @brief class implementing the MLESAC method.
 * @ingroup segmentation
 *
 * The code is a re-factored version of the algorithm implementation in ROS:PCL
 */
//ToDo add reference to the method
class SACMethodLMeDS : public ISACMethods{
public:
	SACMethodLMeDS();
	bool computeModel();
	virtual ~SACMethodLMeDS();
};

}

#endif /* SACMETHODLMEDS_H_ */
