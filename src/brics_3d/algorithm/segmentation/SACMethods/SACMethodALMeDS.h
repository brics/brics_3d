/*
 * @file: SACMethodALMeDS.h
 *
 * @date: Apr 21, 2011
 * @author: reon
 */

#ifndef SACMETHODALMEDS_H_
#define SACMETHODALMEDS_H_

#include "ISACMethods.h"


namespace brics_3d {
//ToDo add reference to the method

/**
 * @ingroup segmentation
 */
class SACMethodALMeDS : public ISACMethods {

public:

	SACMethodALMeDS();
	bool computeModel ();
	virtual ~SACMethodALMeDS();
};

}

#endif /* SACMETHODALMEDS_H_ */
