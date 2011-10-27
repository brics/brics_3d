/*
 * EuclideanClustering3D.h
 *
 *  Created on: Oct 24, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef EUCLIDEANCLUSTERING3D_H_
#define EUCLIDEANCLUSTERING3D_H_

#include "core/PointCloud3D.h"
#include "core/ColoredPointCloud3D.h"
#include "algorithm/nearestNeighbor/NearestNeighborANN.h"
#include "algorithm/nearestNeighbor/NearestNeighborFLANN.h"

#include <vector>
#include<stdio.h>

namespace BRICS_3D {

class EuclideanClustering3D {

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


public:
	EuclideanClustering3D();
	virtual ~EuclideanClustering3D();

	/**
	 * Takes a pointcloud and returns an array of pointcloud that make up the clusters.
	 * The clusters are defined by the parameters being set
	 * @param inCloud	Input point cloud
	 * @param extractedClusters Vector of pointcluds containing the extracted clusters
	 */
	void extractClusters(BRICS_3D::PointCloud3D *inCloud, std::vector<BRICS_3D::PointCloud3D*> *extractedClusters);


	/**
	 * Takes a pointcloud and returns an array of pointcloud that make up the clusters.
	 * The clusters are defined by the parameters being set
	 * @param inCloud	Input point cloud
	 * @param extractedClusters Vector of pointcluds containing the extracted clusters
	 */
	void extractClusters(BRICS_3D::ColoredPointCloud3D *inCloud, std::vector<BRICS_3D::ColoredPointCloud3D*> *extractedClusters);


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

};

}

#endif /* EUCLIDEANCLUSTERING3D_H_ */
