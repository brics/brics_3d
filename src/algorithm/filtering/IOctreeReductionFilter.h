/**
 * @file 
 * IOctreeReductionFilter.h
 *
 * @date: Nov 24, 2009
 * @author: sblume
 */

#ifndef IOCTREEREDUCTIONFILTER_H_
#define IOCTREEREDUCTIONFILTER_H_

#include "core/PointCloud3D.h"

namespace BRICS_3D {

/**
 * @brief Abstract interface for the point cloud size reduction algorithm.
 *
 * This interface allows to reduce the size of a point cloud. The reduced point cloud
 * is approximated by the centers of the leaf cells of an Octree
 *
 * @ingroup filtering
 */
class IOctreeReductionFilter {
public:

	/**
	 * @brief Standard constructor.
	 */
	IOctreeReductionFilter(){};

	/**
	 * @brief Standard destructor.
	 */
	virtual ~IOctreeReductionFilter(){};

	/**
	 * @brief Reduce the size of a point cloud.
	 *
	 * The behavior of the size reduction can be influenced with the parameter voxel size: IOctreeSetup::setVoxelSize()
	 *
	 * @param[in] originalPointCloud The input point cloud that will be reduced. This data will not
	 * be modified.
	 * @param[out] resultPointCloud The new point cloud with the reduced amount of points.
	 */
	virtual void reducePointCloud(PointCloud3D* originalPointCloud, PointCloud3D* resultPointCloud) = 0;


};

}

#endif /* IOCTREEREDUCTIONFILTER_H_ */

/* EOF */
