/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2014, KU Leuven
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

#include "Uuid.h"
#include <sstream>

namespace brics_3d {
namespace rsg {

Uuid::Uuid() {
	this->setToNil();
}

Uuid::~Uuid() {

}

Uuid::Uuid(Uuid& other) {
	*this = other;
}

Uuid::Uuid(const Uuid& other) {
	*this = other;
}

Uuid& Uuid::operator =(Uuid& other) {
	std::copy(other.begin(), other.end(), this->begin());
	return *this;
}

Uuid& Uuid::operator =(const Uuid& other) {
	std::copy(other.begin(), other.end(), this->begin());
	return *this;
}

Uuid& Uuid::operator =(unsigned int other) {
	this->setToNil();
	data[15] = other >> 0;
	data[14] = other >> 8;
	data[13] = other >> 16;
	data[16] = other >> 24;
	return *this;
}

bool Uuid::operator ==(const Uuid& other) const {
	return std::equal(this->begin(), this->end(), other.begin());
}

bool Uuid::operator !=(const Uuid& other) const {
	return !(*this == other);
}

bool Uuid::operator <(const Uuid& other) const {
	return std::lexicographical_compare(this->begin(), this->end(),
			other.begin(), other.end());
}

bool Uuid::operator >(const Uuid& other) const {
	return other < *this;
}

bool Uuid::operator <=(const Uuid& other) const {
	return !(other < *this);
}

bool Uuid::operator >=(const Uuid& other) const {
	return !(*this < other);
}


void Uuid::swap(Uuid& lhs, Uuid& rhs) {
	lhs.swap(rhs);
}

void Uuid::swap(Uuid& other) {
	std::swap_ranges(begin(), end(), other.begin());
}

bool Uuid::isNil() const {
	for(size_t i=0; i<this->size(); i++) {
		if (data[i] != 0U) {
			return false;
		}
	}
	return true;
}

void Uuid::setToNil() {
	for(size_t i=0; i<this->size(); i++) {
		data[i] = 0U;
	}
}

std::string Uuid::toString() const {
	std::stringstream uuidAsSting;
	uuidAsSting.str("");

//	uuidAsSting << std::hex;
	uuidAsSting.fill(uuidAsSting.widen('0'));

     std::size_t i=0;
     for (Uuid::const_iterator i_data = this->begin(); i_data!=this->end(); ++i_data, ++i) {
    	 uuidAsSting.width(2);
    	 uuidAsSting << static_cast<unsigned int>(*i_data);
         if (i == 3 || i == 5 || i == 7 || i == 9) {
        	 uuidAsSting << uuidAsSting.widen('-');
         }
     }

	return uuidAsSting.str();
}

std::ostream& operator<<(std::ostream &outStream, const Uuid &id) {
	outStream << id.toString();

	return outStream;
}

} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
