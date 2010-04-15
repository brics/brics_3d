/**
 * @file 
 * IDelaunayTriangulation.h
 *
 * @date: Feb 26, 2010
 * @author: sblume
 */

#ifndef IDELAUNAYTRIANGULATION_H_
#define IDELAUNAYTRIANGULATION_H_

#include "core/PointCloud3D.h"
#include "core/ITriangleMesh.h"

namespace BRICS_3D {

enum axis {
	x,
	y,
	z
};

class IDelaunayTriangulation {
public:
	IDelaunayTriangulation(){};

	virtual ~IDelaunayTriangulation(){};

	virtual void triangulate(PointCloud3D* pointCloud, ITriangleMesh* mesh, axis ignore = z) = 0;

//	virtual void triangulate(PointCloud3D* pointCloud, ITetrahedronSet* tetrahedrons) = 0;
};

}

#endif /* IDELAUNAYTRIANGULATION_H_ */

/* EOF */
