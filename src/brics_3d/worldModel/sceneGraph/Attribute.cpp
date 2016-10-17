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
#include <boost/regex.hpp>
#include <iostream>
#include "brics_3d/core/Logger.h";

namespace brics_3d {

namespace rsg {

Attribute::Attribute() {

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

}

ostream& operator<<(ostream &outStream, const Attribute &attribute) {
	outStream << "(" << attribute.key << " = " << attribute.value << ")";

	return outStream;
}

extern bool attributeListContainsAttribute(vector<Attribute> attributeList, Attribute queryAttribute) {

	/* Convenience substitution */
	if (queryAttribute.key.compare("*") == 0) {
		queryAttribute.key = "^.*";
	}
	if (queryAttribute.value.compare("*") == 0) {
		queryAttribute.value = "^.*";
	}

	try {
		for (unsigned int i = 0; i < static_cast<unsigned int>(attributeList.size()); ++i) {
			//		if (attributeList[i] == queryAttribute) {
			boost::regex keyExpression(queryAttribute.key);
			boost::regex valueExpression(queryAttribute.value);
			if (regex_match(attributeList[i].key, keyExpression) && regex_match(attributeList[i].value , valueExpression)) {
				return true;
			}
		}
	} catch (std::exception e) {
		return false;
	}
	return false;
}

extern bool getValuesFromAttributeList(vector<Attribute> attributeList, std::string key, vector<std::string>& resultValues) {
	resultValues.clear();
	for(std::vector<Attribute>::iterator it = attributeList.begin(); it != attributeList.end() ;++it) {
		if(it->key.compare(key) == 0) {
			resultValues.push_back(it->value);
		}
	}

	if (resultValues.begin() == resultValues.end()) {
		return false;
	} else {
		return true;
	}

}

extern bool attributeListsAreEqual(vector<Attribute> attributeList1, vector<Attribute> attributeList2) {
//	LOG(DEBUG) << "attributeListsAreEqual: attributeList1: " << attributeList1.size() << " attributeList2: " << attributeList2.size();
	if((attributeList1.size() == 0) && (attributeList2.size() == 0)) {
		return true;
	} else if ((attributeList1.size() == 0) || (attributeList2.size() == 0)) {
		return false;
	} else if (attributeList1.size() != attributeList2.size()) { // short cut
		return false;
	} else if (attributeList1.begin() == attributeList1.end()) {
		std::cout << "attributeListsAreEqual: attributeList1 is empty" << std::endl;
	} else if (attributeList2.begin() == attributeList2.end()) {
		std::cout << "attributeListsAreEqual: attributeList2 is empty" << std::endl;
	}
	return std::equal(attributeList1.begin(), attributeList1.end(), attributeList2.begin());
}

} // namespace brics_3d::RSG

} // namespace brics_3d

/* EOF */

