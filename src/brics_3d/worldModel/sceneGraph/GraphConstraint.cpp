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

	setDefaultValues();

}

GraphConstraint::~GraphConstraint() {

}

bool GraphConstraint::parse(std::string constraintModel) {
	setDefaultValues();

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
	if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint is empty."; return false;}
	if(it->compare("send") == 0) {
		action = SEND;
	} else if(it->compare("receive") == 0) {
		action = RECEIVE;
	} else {
		action = UNDEFINED_ACTION;
	}

	/* Qualifier */
	++it;
	if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no qualifier."; return false;}
	if(it->compare("no") == 0) {
		qualifier = NO;
	} else if(it->compare("only") == 0) {
		qualifier = ONLY;
	} else {
		qualifier = UNDEFINED_QUALIFIER;
	}

	/* Type */
	++it;
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
	} else if(it->compare("Connections") == 0) {
		type = Connection;
	} else {
		type = UNDEFINED_TYPE;
	}

	/*
	 * (( with freq (<|=|>) [0-9]+ (Hz|kHz))|\
	 *  ( with lod (<|=|>) [0-9]+)|\
	 *  ( with dist (<|=|>) [0-9]+ (mm|cm|m|km) from (me|([a-fA-F0-9]{8}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{12}))|\
	 *  ( from context [a-zA-Z0-9]+)|\
	 *  ( contained in ([a-fA-F0-9]{8}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{12}))
	 */

	/* NodeConstraint; here he have to fork the parsing */
	++it;
	if(it == tokens.end()) {
		nodeConstraint = NONE;
		LOG(DEBUG) << "GraphConstraint has no nodeConstraint.";
		return validate();
	}
	if(it->compare("with") == 0) { // FREQUENCY, DISTANCE or LOD

		++it;
		if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has an incomplete freq/lod/dist nodeConstraint."; return false;}
		if(it->compare("freq") == 0) { // FREQUENCY
			nodeConstraint = FREQUENCY;

			// operator: (<|=|>)
			++it;
			if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no freq operator."; return false;}
			if(it->compare("<") == 0) {
				comparision = LT;
			} else if(it->compare("=") == 0) {
				comparision = EQ;
			} else if(it->compare(">") == 0) {
				comparision = GT;
			} else {
				LOG(ERROR) << "GraphConstraint has an invalid freq operator: " << *it;;
				return false;
			}

			// value: [0-9]+
			++it;
			if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no value."; return false;}
			value = atof(it->c_str());

			// unit: (Hz|kHz)
			++it;
			if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no freq unit."; return false;}
			if(it->compare("Hz") == 0) {
				freqUnit = Units::Hertz;
			} else if(it->compare("kHz") == 0) {
				freqUnit = Units::KiloHertz;
			} else {
				LOG(ERROR) << "GraphConstraint has an invalid freq unit: " << *it;
				return false;
			}


		} else if(it->compare("lod") == 0) { // LOD
			nodeConstraint = LOD;

			// operator: (<|=|>)
			++it;
			if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no lod operator."; return false;}
			if(it->compare("<") == 0) {
				comparision = LT;
			} else if(it->compare("=") == 0) {
				comparision = EQ;
			} else if(it->compare(">") == 0) {
				comparision = GT;
			} else {
				LOG(ERROR) << "GraphConstraint has an invalid lod operator: " << *it;;
				return false;
			}

			// value: [0-9]+
			++it;
			if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no lod value."; return false;}
			value = atof(it->c_str());

		} else if(it->compare("dist") == 0) { // DISTANCE
			nodeConstraint = DISTANCE;

			// operator: (<|=|>)
			++it;
			if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no dist operator."; return false;}
			if(it->compare("<") == 0) {
				comparision = LT;
			} else if(it->compare("=") == 0) {
				comparision = EQ;
			} else if(it->compare(">") == 0) {
				comparision = GT;
			} else {
				LOG(ERROR) << "GraphConstraint has an invalid dist operator: " << *it;;
				return false;
			}

			// value: [0-9]+
			++it;
			if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no dust value."; return false;}
			value = atof(it->c_str());

			// unit: (mm|cm|m|km)
			++it;
			if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no dist unit."; return false;}
			if(it->compare("mm") == 0) {
				distUnit = Units::MilliMeter;
			} else if(it->compare("cm") == 0) {
				distUnit = Units::CentiMeter;
			} else if(it->compare("m") == 0) {
				distUnit = Units::Meter;
			} else if(it->compare("km") == 0) {
				distUnit = Units::KiloMeter;
			} else {
				LOG(ERROR) << "GraphConstraint has an invalid dist unit: " << *it;
				return false;
			}

			// from
			++it;
			if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no from token."; return false;}
			if(it->compare("from") != 0) { LOG(ERROR) << "GraphConstraint has " << *it << " instead of from token."; return false;}

			// (me|([a-fA-F0-9]{8}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{12}))
			++it;
			if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no nodeId token."; return false;}
			if(it->compare("me") == 0) {
				isMe = true;
			} else {
				node.fromString(*it);
			}


		} else {
			LOG(ERROR) << "GraphConstraint has an invalid freq/lod/dist nodeConstraint.";
			return false;
		}

	} else if(it->compare("from") == 0) {
		nodeConstraint = CONTEXT;

		// context
		++it;
		if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no context token."; return false;}
		if(it->compare("context") != 0) { LOG(ERROR) << "GraphConstraint has " << *it << " instead of context token."; return false;}

		// [a-zA-Z0-9]+
		++it;
		if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no context definition."; return false;}
		context = *it;

	} else if(it->compare("contained") == 0) {
		nodeConstraint = CONTAINMENT;

		// in
		++it;
		if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no in token."; return false;}
		if(it->compare("in") != 0) { LOG(ERROR) << "GraphConstraint has " << *it << " instead of in token."; return false;}

		// ([a-fA-F0-9]{8}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{12})
		++it;
		if(it == tokens.end()){ LOG(ERROR) << "GraphConstraint has no nodeId"; return false;}
		if(it->compare("me") == 0) { // this is actually not specified, but will most likely come soon...
			isMe = true;
		} else {
			node.fromString(*it);
		}

	} else {
		nodeConstraint = UNDEFINED_NODE_CONSTRAINT;
		LOG(ERROR) << "GraphConstraint has an invalid nodeConstraint.";
		return false;
	}

	return validate();
}

bool GraphConstraint::validate() {
	return (qualifier != UNDEFINED_QUALIFIER)
			&& (type != UNDEFINED_TYPE);
}

void GraphConstraint::setDefaultValues() {

	action = UNDEFINED_ACTION;
	qualifier = UNDEFINED_QUALIFIER;
	type = UNDEFINED_TYPE;
	comparision = UNDEFINED_OPERATOR;
	value = 0;
	freqUnit = Units::Hertz;
	distUnit = Units::Meter;
	node = 0;
	isMe = false;
	context = "";

}

} /* namespace rsg */
} /* namespace brics_3d */


/* EOF */
