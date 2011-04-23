/*
 * SACMethodALMeDS.h
 *
 *  Created on: Apr 21, 2011
 *      Author: reon
 */

#ifndef SACMETHODALMEDS_H_
#define SACMETHODALMEDS_H_

#include "ISACMethods.h"


namespace BRICS_3D {

class SACMethodALMeDS : public ISACMethods {

public:

	SACMethodALMeDS();
	bool computeModel ();
	virtual ~SACMethodALMeDS();
};

}

#endif /* SACMETHODALMEDS_H_ */
