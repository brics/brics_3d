/**
 * @file 
 * DelaunayTriangulationOSG.h
 *
 * @date: Feb 26, 2010
 * @author: sblume
 */

#ifndef DELAUNAYTRIANGULATIONOSG_H_
#define DELAUNAYTRIANGULATIONOSG_H_

#include "IDelaunayTriangulation.h"

namespace BRICS_3D {

class DelaunayTriangulationOSG : public IDelaunayTriangulation {
public:
	DelaunayTriangulationOSG();

	virtual ~DelaunayTriangulationOSG();

	void triangulate(PointCloud3D* pointCloud, ITriangleMesh* mesh, axis ignore = z);
};

}

#endif /* DELAUNAYTRIANGULATIONOSG_H_ */

/* EOF */
