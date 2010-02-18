/**
 * @file 
 * Octree.h
 *
 * @date: Feb 18, 2010
 * @author: sblume
 */

#ifndef OCTREE_H_
#define OCTREE_H_

#include "algorithm/filter/IOctree.h"
#include "algorithm/filter/IOctreeSetup.h"

namespace BRICS_3D {

class Octree : public IOctree, public IOctreeSetup {
public:
	Octree();

	virtual ~Octree();

	void createOctree(PointCloud3D* originalPointCloud, PointCloud3D* resultPointCloud);

	void setVoxelSize(double voxelSize);

	double getVoxelSize();

private:

	double voxelSize;

};

}

#endif /* OCTREE_H_ */

/* EOF */
