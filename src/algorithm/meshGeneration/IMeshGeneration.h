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

#ifndef IMESHGENERATION_H_
#define IMESHGENERATION_H_

#include "core/PointCloud3D.h"
#include "core/ITriangleMesh.h"

namespace BRICS_3D {

/**
 * @brief Generic interface for a triangle mesh generation componennt.
 * @ingroup mesh_generation
 */
class IMeshGeneration {
public:
	IMeshGeneration(){};
	virtual ~IMeshGeneration(){};

	virtual void generateMesh(PointCloud3D* pointCloud, ITriangleMesh* mesh) = 0;
};

}

#endif /* IMESHGENERATION_H_ */

/* EOF */
