/******************************************************************************
* BRICS_3D - 3D Perception and Modeling Library
* Copyright (c) 2011, GPS GmbH
*
* Author: Pinaki Sunil Banerjee
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

#ifndef BRICS_3D_EUCLIDEANCLUSTERING3D_H_
#define BRICS_3D_EUCLIDEANCLUSTERING3D_H_

#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/algorithm/nearestNeighbor/NearestNeighborANN.h"
#include "brics_3d/algorithm/nearestNeighbor/NearestNeighborFLANN.h"
#include "brics_3d/algorithm/segmentation/ISegmentation.h"

#include <vector>
#include<stdio.h>

namespace brics_3d {

/**
 * @brief Segmentation based on Eucledian distance between point clusters.
 * @ingroup segmentation
 */
class EuclideanClustering : public ISegmentation{

private:

	/**
	 * spacial cluster tolerance in metres
	 */
	float clusterTolerance;


	/**
	 * Minimum number of points to consider it as a cluster
	 */
	unsigned int minClusterSize;


	/**
	 * Maximum number of points to be in the cluster
	 */
	unsigned int maxClusterSize;

	/**
	 * Takes a pointcloud and returns an array of pointcloud that make up the clusters.
	 * The clusters are defined by the parameters being set
	 * @param inCloud	Input point cloud
	 * @param extractedClusters Vector of pointcluds containing the extracted clusters
	 */
	void extractClusters(brics_3d::PointCloud3D *inCloud);


	std::vector<brics_3d::PointCloud3D*> extractedClusters;


public:
	EuclideanClustering();
	virtual ~EuclideanClustering();


	/**
	 * @return the used spatial cluster tolerance
	 */
	float getClusterTolerance() const
	{
		return clusterTolerance;
	}


	/**
	 *
	 * @return maximum no of point in a cluster till the cluster is valid
	 */
	int getMaxClusterSize() const
	{
		return maxClusterSize;
	}


	/**
	 *
	 * @return minimum no of point in a cluster till the cluster is valid
	 */
	int getMinClusterSize() const
	{
		return minClusterSize;
	}


	/**
	 *
	 * @param clusterTolerance spacial cluster tolerance
	 */
	void setClusterTolerance(float clusterTolerance)
	{
		this->clusterTolerance = clusterTolerance;
	}


	/**
	 *
	 * @param maxClusterSize maximum no of point in a cluster till the cluster is valid
	 */
	void setMaxClusterSize(int maxClusterSize)
	{
		this->maxClusterSize = maxClusterSize;
	}


	/**
	 *
	 * @param minClusterSize minimum no of point in a cluster till the cluster is valid
	 */
	void setMinClusterSize(int minClusterSize)
	{
		this->minClusterSize = minClusterSize;
	}


	void getExtractedClusters(std::vector<brics_3d::PointCloud3D*> &extractedClusters){
		extractedClusters = this->extractedClusters;
	}

	int segment();
};

}

#endif /* BRICS_3D_EUCLIDEANCLUSTERING3D_H_ */
