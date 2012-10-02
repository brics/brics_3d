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

#ifndef DELAUNAYTRIANGULATIONOSG_H_
#define DELAUNAYTRIANGULATIONOSG_H_

#include "IDelaunayTriangulation.h"
#include "IMeshGeneration.h"

namespace brics_3d {

/**
 * @brief Implementation for 2.5D Delaunay triangulation method offered from OSG library.
 * @ingroup mesh_generation
 *
 * This algorithm takes a point cloud as input and flattens it along one axis. This
 * projection is used to perform a Delaunay triangulation in 2D space.
 *
 */
class DelaunayTriangulationOSG : public IDelaunayTriangulation, public IMeshGeneration {
public:
	DelaunayTriangulationOSG();

	virtual ~DelaunayTriangulationOSG();

	void triangulate(PointCloud3D* pointCloud, ITriangleMesh* mesh, axis ignore = z);

	void generateMesh(PointCloud3D* pointCloud, ITriangleMesh* mesh);
};

}

#endif /* DELAUNAYTRIANGULATIONOSG_H_ */

/* EOF */
