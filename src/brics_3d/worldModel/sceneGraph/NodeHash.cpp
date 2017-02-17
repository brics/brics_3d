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

#include "NodeHash.h"
#include "hash/sha256.h"

#include <sstream>

namespace brics_3d {
namespace rsg {

NodeHash::NodeHash() {

}

NodeHash::~NodeHash() {

}

std::string NodeHash::idToHash(Id id) {
	SHA256 sha256;
	sha256(id.toString());
	return sha256.getHash();
}

std::string NodeHash::attributesToHash(std::vector<Attribute> attributes) {
	SHA256 sha256;
	sha256(attributeListToString(attributes));
	return sha256.getHash();
}

std::string NodeHash::sortStringsAndHash(std::vector<std::string> strings) {
	std::sort(strings.begin(), strings.end()); //lexicographical sort to be independent of order
	std::stringstream ss;
	for (std::vector<std::string>::iterator it = strings.begin(); it != strings.end(); ++it) {
		ss << *it;
	}
	SHA256 sha256;
	sha256(ss.str());
	return sha256.getHash();
}


} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
