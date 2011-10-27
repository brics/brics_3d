/******************************************************************************
* Copyright (c) 2011
* GPS GmbH
*
* Author:
* Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of GPS GmbH nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include "Attribute.h"

namespace BRICS_3D {

namespace RSG {




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

extern bool attributeListContainsAttribute(Attribute queryAttribute, vector<Attribute>& attributeList) {
	for (unsigned int i = 0; i < static_cast<unsigned int>(attributeList.size()); ++i) {
		if (attributeList[i] == queryAttribute) {
			return true;
		}
	}
	return false;
}

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D

/* EOF */

