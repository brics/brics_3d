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

#ifndef RSG_ID_H_
#define RSG_ID_H_

#include "Uuid.h"

#define NO_UUID

namespace brics_3d {
namespace rsg {

/// The Id uniquely identifies a node in the scene graph.
#ifdef NO_UUID
	///@deprecated
	typedef unsigned int Id;
#else
	typedef Uuid Id;
#endif

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* RSG_ID_H_ */
