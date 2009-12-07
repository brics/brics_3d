/**
 * @file 
 * CorrespondenceIndexPair.cpp
 *
 * @date: Dec 3, 2009
 * @author: sblume
 */

#include "CorrespondenceIndexPair.h"

namespace BRICS_3D {

CorrespondenceIndexPair::CorrespondenceIndexPair() {

}

CorrespondenceIndexPair::~CorrespondenceIndexPair() {

}

CorrespondenceIndexPair::CorrespondenceIndexPair(unsigned int firstIndex, unsigned int secondIndex){
	this->firstIndex = firstIndex;
	this->secondIndex = secondIndex;
}

}

/* EOF */
