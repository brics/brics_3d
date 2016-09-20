/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2016, KU Leuven
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

#include "GraphConstraint.h"
#include <brics_3d/core/Logger.h>
#include <boost/algorithm/string.hpp>
#include <vector>

namespace brics_3d {
namespace rsg {

GraphConstraint::GraphConstraint() {

	action = UNDEFINED_ACTION;
	qualifier = UNDEFINED_QUALIFIER;
	type = UNDEFINED_TYPE;
	comparision = UNDEFINED_OPERATOR;
	value = 0;
	freqUnit = Units::Hertz;
	distUnit = Units::Meter;
	node = 0;
	context = "";

}

GraphConstraint::~GraphConstraint() {

}

bool GraphConstraint::parse(std::string constraintModel) {

	/*
	 * Check if model is valid
	 */

	/*
	 * Parse the model
	 */

	std::vector<std::string> tokens;
	boost::split(tokens, constraintModel, boost::is_any_of(" "));
	std::vector<std::string>::const_iterator it = tokens.begin();

	/* Action */
	if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint is empty." ; return false;}
	if(it->compare("send") == 0) {
		action = SEND;
	} else if(it->compare("receive") == 0) {
		action = RECEIVE;
	} else {
		action = UNDEFINED_ACTION;
	}

	/* Qualifier */
	if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no qualifier." ; return false;}
	if(it->compare("no") == 0) {
		qualifier = NO;
	} else if(it->compare("only") == 0) {
		qualifier = ONLY;
	} else {
		qualifier = UNDEFINED_QUALIFIER;
	}

	/* Type */
	if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no type." ; return false;}
	if(it->compare("Atoms") == 0) {
		type = Atom;
	} else if(it->compare("Nodes") == 0) {
		type = Node;
	} else if(it->compare("GeometricNodes") == 0) {
		type = GeometricNode;
	} else if(it->compare("Spheres") == 0) {
		type = Sphere;
	} else if(it->compare("Cylinders") == 0) {
		type = Cylinder;
	} else if(it->compare("Boxes") == 0) {
		type = Box;
	} else if(it->compare("PointClouds") == 0) {
		type = PointCloud;
	} else if(it->compare("Meshes") == 0) {
		type = Mesh;
	} else if(it->compare("Transforms") == 0) {
		type = Transform;
	} else if(it->compare("Groups") == 0) {
		type = Group;
	} else if(it->compare("Connection") == 0) {
		type = Connection;
	} else {
		type = UNDEFINED_TYPE;
	}

	/* NodeConstraint; here he have to fork the parsing */

	return validate();
}

bool GraphConstraint::validate() {
	return (qualifier != UNDEFINED_QUALIFIER)
			&& (type != UNDEFINED_TYPE);
}


} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
