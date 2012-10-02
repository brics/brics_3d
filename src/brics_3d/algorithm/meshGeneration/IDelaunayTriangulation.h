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

#ifndef BRICS_3D_IDELAUNAYTRIANGULATION_H_
#define BRICS_3D_IDELAUNAYTRIANGULATION_H_

#include "core/PointCloud3D.h"
#include "core/ITriangleMesh.h"

namespace brics_3d {

enum axis {
	x,
	y,
	z
};

/**
 * @brief Abstract interface for Delaunay triangulation methods
 * @ingroup mesh_generation
 */
class IDelaunayTriangulation {
public:
	IDelaunayTriangulation(){};

	virtual ~IDelaunayTriangulation(){};

	virtual void triangulate(PointCloud3D* pointCloud, ITriangleMesh* mesh, axis ignore = z) = 0;

//	virtual void triangulate(PointCloud3D* pointCloud, ITetrahedronSet* tetrahedrons) = 0;


};

}

#endif /* BRICS_3D_IDELAUNAYTRIANGULATION_H_ */

/* EOF */
