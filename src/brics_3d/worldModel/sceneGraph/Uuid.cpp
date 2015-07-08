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
#include "brics_3d/core/Logger.h"
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

Uuid::Uuid(unsigned int id) {
	*this = id;
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
	data[12] = other >> 24;
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

	uuidAsSting << std::hex;
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

bool Uuid::fromString(std::string uuid) {
	const size_t expectedStringSize = 36; // 32 digits and four "-" hyphens
	std::stringstream is(uuid);
	typedef std::ctype<char> ctype_t;
	ctype_t const& ctype = std::use_facet<ctype_t>(is.getloc());

	if(uuid.size() != expectedStringSize) {
		LOG(ERROR) << "Failed to parse UUID. Expected string size of " << expectedStringSize << " does not match actual string size of "<< uuid.size() << ".";
		this->setToNil();
		return false;
	}

	char xdigits[16];
	{
		char szdigits[] = "0123456789ABCDEF";
		ctype.widen(szdigits, szdigits+16, xdigits);
	}
	char*const xdigits_end = xdigits+16;

	char c;
	for (std::size_t i=0; i<this->arraySize && is; ++i) {
		is >> c;
		c = ctype.toupper(c);

		char* f = std::find(xdigits, xdigits_end, c);
		if (f == xdigits_end) {
			LOG(ERROR) << "Failed to parse UUID with character " << c << " at position "<< i << ".";
			this->setToNil();
			return false;
		}

		unsigned char byte = static_cast<unsigned char>(std::distance(&xdigits[0], f));

		is >> c;
		c = ctype.toupper(c);
		f = std::find(xdigits, xdigits_end, c);
		if (f == xdigits_end) {
			LOG(ERROR) << "Failed to parse UUID with character " << c << " at position "<< i << ".";
			this->setToNil();
			return false;
		}

		byte <<= 4;
		byte |= static_cast<unsigned char>(std::distance(&xdigits[0], f));

		this->data[i] = byte;

		if (is) {
			if (i == 3 || i == 5 || i == 7 || i == 9) {
				is >> c;
				if (c != is.widen('-')) {
					LOG(ERROR) << "Failed to parse UUID. Expected a '-' on position " << i <<" but retrieved chracter " << c << " instead." ;
					this->setToNil();
					return false;
				}
			}
		}
	}

	return true;

}

std::ostream& operator<<(std::ostream &outStream, const Uuid &id) {
	outStream << id.toString();

	return outStream;
}

unsigned int uuidToUnsignedInt(Uuid id) {
	unsigned int uuidAsInt = 0u;
	const int startByte = 12; // Cut away the first 12 bytes (0-11)

//	unsigned int value = (unsigned int)*p <<24 |
//	                     (unsigned int)*(p+1) << 16 |
//	                     (unsigned int)*(p+2) << 8 |
//	                     (unsigned int)*(p+3);

	int bitShift = 24; // 24, 16, 8, 0
	for (Uuid::const_iterator i_data = id.begin()+startByte; i_data!=id.end(); ++i_data) {
		uuidAsInt = uuidAsInt | (unsigned int)*i_data << bitShift;
		bitShift -= 8;
	}

	return uuidAsInt;
}

unsigned int uuidToUnsignedInt (unsigned int id) {
	return id;
}

} /* namespace rsg */
} /* namespace brics_3d */


/* EOF */
