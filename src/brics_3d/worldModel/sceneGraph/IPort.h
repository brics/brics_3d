/******************************************************************************
 * BRICS_3D - 3D Perception and Modeling Library
 * Copyright (c) 2015, KU Leuven
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

#ifndef RSG_IPORT_H_
#define RSG_IPORT_H_

namespace brics_3d {
namespace rsg {

/**
 * @brief Generic output port interface for serialized scene graph updates.
 */
class IOutputPort {
public:
	IOutputPort(){};
	virtual ~IOutputPort(){};
	virtual int write(const char *dataBuffer, int dataLength, int &transferredBytes) = 0;
};

/**
 * @brief Generic output input port interface for serialized scene graph updates.
 */
class IInputPort {
public:
	IInputPort(){};
	virtual ~IInputPort(){};
	virtual int write(const char *dataBuffer, int dataLength, int &transferredBytes) = 0; // triggers update of WM
};

} /* namespace rsg */
} /* namespace brics_3d */

#endif /* IPORT_H_ */

/* EOF */
