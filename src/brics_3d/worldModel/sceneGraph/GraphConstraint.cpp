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


	return validate();
}

bool GraphConstraint::validate() {
	return (qualifier != UNDEFINED_QUALIFIER)
			&& (type != UNDEFINED_TYPE);
}


} /* namespace rsg */
} /* namespace brics_3d */

/* EOF */
