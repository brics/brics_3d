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

class IDelaunayTriangulation {
public:
	IDelaunayTriangulation(){};

	virtual ~IDelaunayTriangulation(){};

	virtual void triangulate(PointCloud3D* pointCloud, ITriangleMesh* mesh) = 0;
};

}

#endif /* IDELAUNAYTRIANGULATION_H_ */

/* EOF */
