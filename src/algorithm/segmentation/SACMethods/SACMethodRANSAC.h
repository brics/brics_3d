/*
 * @file: SACMethodRANSAC.h
 *
 * @date: Apr 19, 2011
 * @author: reon
 */

#ifndef SACMETHODRANSAC_H_
#define SACMETHODRANSAC_H_

#include "ISACMethods.h"


namespace brics_3d {

/**
 * @brief class implementing the Random Sample Consensus based segmentation method.
 * @ingroup segmentation
 *
 * The code is a re-factored version of the algorithm implementation in ROS:PCL.
 *
 **/
//ToDo add reference to the method
class SACMethodRANSAC : public ISACMethods {

public:

	SACMethodRANSAC();
	bool computeModel ();
	virtual ~SACMethodRANSAC();
};

}

#endif /* SACMETHODRANSAC_H_ */
