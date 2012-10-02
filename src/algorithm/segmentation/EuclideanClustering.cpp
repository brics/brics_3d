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

#include "EuclideanClustering.h"
#include <iostream>
#include <algorithm>
namespace brics_3d {

EuclideanClustering::EuclideanClustering() {
	// TODO Auto-generated constructor stub

}

EuclideanClustering::~EuclideanClustering() {
	// TODO Auto-generated destructor stub
}

void EuclideanClustering::extractClusters(brics_3d::PointCloud3D *inCloud){


	int k = inCloud->getSize();
	brics_3d::NearestNeighborANN nearestneighborSearch;
	brics_3d::Point3D querryPoint3D;
	vector<int> neighborIndices;

	nearestneighborSearch.setData(inCloud);
	nearestneighborSearch.setMaxDistance(this->clusterTolerance);
	//	nearestneighborSearch.setMaxDistance(1000);
	// Create a bool vector of processed point indices, and initialize it to false
	std::vector<bool> processed (inCloud->getSize(), false);


	// Process all points in the indices vector
	for (size_t i = 0; i < inCloud->getSize(); ++i)
	{
		if (processed[i])
			continue;

		std::vector<int> seed_queue;
		int sq_idx = 0;
		seed_queue.push_back (i);
		//std::cout << "[CHEAT][EuclideanClustering3D] "<< i << " : " << seed_queue.size () << std::endl;
		processed[i] = true;

		while (sq_idx < (int)seed_queue.size ())
		{

			// Search for sq_idx
			neighborIndices.clear();
			querryPoint3D = (*inCloud->getPointCloud())[sq_idx];
			nearestneighborSearch.findNearestNeighbors(&querryPoint3D, &neighborIndices, k);

			//if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
			if(neighborIndices.size()==0)
			{
				sq_idx++;
				continue;
			}

			for (size_t j = 0; j < neighborIndices.size(); ++j)             // nn_indices[0] should be sq_idx
			{
				if (processed[neighborIndices[j]])                             // Has this point been processed before ?
					continue;

				// Perform a simple Euclidean clustering
				seed_queue.push_back (neighborIndices[j]);
				processed[neighborIndices[j]] = true;
			}

			sq_idx++;
		}
//		std::cout << "[CHEAT][EuclideanClustering3D] {sq_id : seed_q.size}"<< sq_idx << " : "<< seed_queue.size() << std::endl;
		// If this queue is satisfactory, add to the clusters
		if (seed_queue.size () >= minClusterSize && seed_queue.size () <= maxClusterSize)
		{
//			std::cout << "[CHEAT][EuclideanClustering3D] found one cluster, size="<< seed_queue.size() << std::endl;

			seed_queue.erase(std::unique(seed_queue.begin(), seed_queue.end()),seed_queue.end());
			brics_3d::PointCloud3D *tempPointCloud =  new brics_3d::PointCloud3D();

			for (size_t j = 0; j < seed_queue.size (); ++j) {

				tempPointCloud->addPointPtr((*inCloud->getPointCloud())[seed_queue[j]].clone());

			}
			extractedClusters.push_back(tempPointCloud);
//			delete tempPointCloud; //?
		}
//		std::cout << "[CHEAT][EuclideanClustering3D] "<< i << " : " << seed_queue.size () << std::endl;
	}
}



int EuclideanClustering::segment(){

	assert (this->inputPointCloud != 0);
	extractClusters(this->inputPointCloud);
	return 1;
}
}
