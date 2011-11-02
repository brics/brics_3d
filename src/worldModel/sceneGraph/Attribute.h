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

#ifndef ATTRIBUTE_H
#define ATTRIBUTE_H

#include <string>
#include <vector>
#include <iostream>
using std::ostream;
using std::istream;
using std::string;
using std::vector;

namespace BRICS_3D {

namespace RSG {

class Attribute {
  public:
    string key;

    string value;

//     operator<<();

    Attribute (string key, string value);
//
//    Attribute(Attribute & source);
//
//    Attribute(const Attribute & source);
//
//    Attribute & operator=(Attribute & source);
//
//    Attribute & operator=(const Attribute & source);

    bool operator==(Attribute& other);

    bool operator==(const Attribute& other);

    bool operator!=(Attribute& other);

    bool operator!=(const Attribute& other);

    Attribute();

    virtual ~Attribute();

	friend ostream& operator<<(ostream &outStream, const Attribute &attribute);

};

extern bool attributeListContainsAttribute(vector<Attribute> attributeList, Attribute queryAttribute);

} // namespace BRICS_3D::RSG

} // namespace BRICS_3D
#endif

/* EOF */

