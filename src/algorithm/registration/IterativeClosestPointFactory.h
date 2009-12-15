/**
 * @file 
 * IterativeClosestPointFactory.h
 *
 * @date: Dec 15, 2009
 * @author: sblume
 */

#ifndef ITERATIVECLOSESTPOINTFACTORY_H_
#define ITERATIVECLOSESTPOINTFACTORY_H_

#include "algorithm/registration/IIterativeClosestPoint.h"
#include <string>


namespace BRICS_3D {

class IterativeClosestPointFactory {
public:
	IterativeClosestPointFactory();
	virtual ~IterativeClosestPointFactory();

	IIterativeClosestPoint* createIterativeClosestPoint(); //TODO shared pointer
	IIterativeClosestPoint* createIterativeClosestPoint(std::string configurationFile); //TODO shared pointer
};

}

#endif /* ITERATIVECLOSESTPOINTFACTORY_H_ */

/* EOF */
