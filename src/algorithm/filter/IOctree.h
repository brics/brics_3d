/**
 * @file 
 * IOctree.h
 *
 * @date: Nov 24, 2009
 * @author: sblume
 */

#ifndef IOCTREE_H_
#define IOCTREE_H_

#include "core/PointCloud3D.h"

namespace BRICS_3D {


/**
 * @brief Abstract interface for the point cloud size reduction algorithm.
 *
 * This interface allows to partition a point cloud into several point cloud with the help of an Octree.
 * Each leaf cell of the octree forms a partition.
 *
 * @ingroup filtering
 */
class IOctree {
public:
	IOctree(){};

	virtual ~IOctree(){};

	/**
	 * @brief Create a reduced point cloud
	 */
	virtual void createOctree(PointCloud3D* originalPointCloud, PointCloud3D* resultPointCloud) = 0;



};

}

#endif /* IOCTREE_H_ */

/* EOF */
