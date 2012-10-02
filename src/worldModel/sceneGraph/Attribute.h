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

namespace brics_3d {

namespace rsg {

/**
 * Attribute lists store (string-based) key-value pairs. Each node has attached attributes.
 * So it is possible to start a search query on all nodes in a scenegraph and identify
 * those nodes specified by the attributes. This concept can be seen as "semantic tagging"
 * of the elements in a scene. These tags could contain something generic like a name or
 * a node type or task specific tags like (color, green) or "robot" or "obstacle", etc.
 *
 * For a particular application a convention for these attributes need to be introduced.
 * Such an example could be:
 *
 *   - "shapeType": {"Box"}
 *   - "color": {"red", "green", "yellow"}
 *
 * @ingroup sceneGraph
 */
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

/**
 * Helper tool to check if a certain attribute is included in an attribute list.
 * @param attributeList The list to be checked.
 * @param queryAttribute THe attribute of intrest.
 * @return True if list contains the attribute otherwise false.
 */
extern bool attributeListContainsAttribute(vector<Attribute> attributeList, Attribute queryAttribute);

} // namespace brics_3d::RSG

} // namespace brics_3d
#endif

/* EOF */

