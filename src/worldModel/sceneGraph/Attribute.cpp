/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#include "Attribute.h"

namespace brics_3d {

namespace rsg {




// Attribute::operator<<() {
//  // Bouml preserved body begin 00025E83
//  // Bouml preserved body end 00025E83
//}

//Attribute::Attribute(Attribute & source) {
//  // Bouml preserved body begin 0002CF83
//  // Bouml preserved body end 0002CF83
//}
//
//Attribute::Attribute(const Attribute & source) {
//  // Bouml preserved body begin 0002D003
//  // Bouml preserved body end 0002D003
//}
//
//Attribute & Attribute::operator=(Attribute & source) {
//  // Bouml preserved body begin 0002D083
//  // Bouml preserved body end 0002D083
//}
//
//Attribute & Attribute::operator=(const Attribute & source) {
//  // Bouml preserved body begin 0002D103
//  // Bouml preserved body end 0002D103
//}

Attribute::Attribute() {
  // Bouml preserved body begin 0002DC03
  // Bouml preserved body end 0002DC03
}

Attribute::Attribute(string key, string value) {
	this->key = key;
	this->value = value;
}

bool Attribute::operator==(Attribute& other) {
	return ( (this->key.compare(other.key) == 0) && (this->value.compare(other.value) == 0) );
}

bool Attribute::operator==(const Attribute& other){
	return ( (this->key.compare(other.key) == 0) && (this->value.compare(other.value) == 0) );
}

bool Attribute::operator!=(Attribute& other) {
	return !(*this == other);
}

bool Attribute::operator!=(const Attribute& other){
	return !(*this == other);
}

Attribute::~Attribute() {
  // Bouml preserved body begin 0002DC83
  // Bouml preserved body end 0002DC83
}

ostream& operator<<(ostream &outStream, const Attribute &attribute) {
	outStream << "(" << attribute.key << " = " << attribute.value << ")";

	return outStream;
}

extern bool attributeListContainsAttribute(vector<Attribute> attributeList, Attribute queryAttribute) {
	for (unsigned int i = 0; i < static_cast<unsigned int>(attributeList.size()); ++i) {
		if (attributeList[i] == queryAttribute) {
			return true;
		}
	}
	return false;
}


} // namespace brics_3d::RSG

} // namespace brics_3d

/* EOF */

