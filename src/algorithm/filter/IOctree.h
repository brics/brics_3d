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
 * @brief Abstract interface for the octree size reduction algorithm.
 * @ingroup filtering
 *
 * More information can be found here: http://en.wikipedia.org/wiki/Octree
 *
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
