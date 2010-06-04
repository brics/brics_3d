/**
 * @file 
 * IOctreePartition.h
 *
 * @date: Mar 22, 2010
 * @author: sblume
 */

#ifndef IOCTREEPARTITION_H_
#define IOCTREEPARTITION_H_

#include "core/PointCloud3D.h"
#include <vector>

namespace BRICS_3D {

/**
 * @brief Abstract interface for spatial partition of point clouds.
 *
 * This interface allows to partition a point cloud into several point cloud with the help of an Octree.
 * Each leaf cell of the octree forms a partition.
 *
 * @ingroup filtering
 */
class IOctreePartition {
public:

	/**
	 * @brief Standard constructor.
	 */
	IOctreePartition(){};

	/**
	 * @brief Standard destructor.
	 */
	virtual ~IOctreePartition(){};

	/**
	 * @brief Partitions a point cloud into sub points clouds.
	 *
	 * The behavior of the subdivision can be influenced with the parameter voxel size: IOctreeSetup::setVoxelSize()
	 *
	 * @param[in] pointCloud The input point cloud that will be partitioned. This data will not
	 * be modified.
	 * @param[out] pointCloudCells Resulting vector of point clouds. Each point cloud represents one cell of the partition.
	 *
	 */
	virtual void partitionPointCloud(PointCloud3D* pointCloud, std::vector<PointCloud3D*>* pointCloudCells) = 0;
};

}

#endif /* IOCTREEPARTITION_H_ */

/* EOF */
