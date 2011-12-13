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

#include "EuclideanClustering3D.h"
#include <iostream>
#include <algorithm>
namespace BRICS_3D {

EuclideanClustering3D::EuclideanClustering3D() {
	// TODO Auto-generated constructor stub

}

EuclideanClustering3D::~EuclideanClustering3D() {
	// TODO Auto-generated destructor stub
}

void EuclideanClustering3D::extractClusters(BRICS_3D::PointCloud3D *inCloud){


	int k = inCloud->getSize();
	BRICS_3D::NearestNeighborANN nearestneighborSearch;
	BRICS_3D::Point3D querryPoint3D;
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
			querryPoint3D = inCloud->getPointCloud()->data()[sq_idx];
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
		std::cout << "[CHEAT][EuclideanClustering3D] {sq_id : seed_q.size}"<< sq_idx << " : "<< seed_queue.size() << std::endl;
		// If this queue is satisfactory, add to the clusters
		if (seed_queue.size () >= minClusterSize && seed_queue.size () <= maxClusterSize)
		{
			std::cout << "[CHEAT][EuclideanClustering3D] found one cluster, size="<< seed_queue.size() << std::endl;

			seed_queue.erase(std::unique(seed_queue.begin(), seed_queue.end()));
			BRICS_3D::PointCloud3D *tempPointCloud =  new BRICS_3D::PointCloud3D();

			for (size_t j = 0; j < seed_queue.size (); ++j) {

				BRICS_3D::Point3D *tempPoint =  new BRICS_3D::Point3D(inCloud->getPointCloud()->data()[seed_queue[j]].getX(),
						inCloud->getPointCloud()->data()[seed_queue[j]].getY(),
						inCloud->getPointCloud()->data()[seed_queue[j]].getZ());
				tempPointCloud->addPoint(tempPoint);


				delete tempPoint;
			}
			extractedClusters.push_back(tempPointCloud);
			delete tempPointCloud;
		}
		std::cout << "[CHEAT][EuclideanClustering3D] "<< i << " : " << seed_queue.size () << std::endl;
	}
}
void EuclideanClustering3D::extractClusters(BRICS_3D::ColoredPointCloud3D *inCloud){
	printf("Nothing is implemented for Colored Point CLouds\n");


	int k = inCloud->getSize();
	BRICS_3D::NearestNeighborANN nearestneighborSearch;
	BRICS_3D::Point3D querryPoint3D;
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
			querryPoint3D = inCloud->getPointCloud()->data()[sq_idx];
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
		std::cout << "[CHEAT][EuclideanClustering3D] {sq_id : seed_q.size}"<< sq_idx << " : "<< seed_queue.size() << std::endl;
		// If this queue is satisfactory, add to the clusters
		if (seed_queue.size () >= minClusterSize && seed_queue.size () <= maxClusterSize)
		{
			std::cout << "[CHEAT][EuclideanClustering3D] found one cluster, size="<< seed_queue.size() << std::endl;

			seed_queue.erase(std::unique(seed_queue.begin(), seed_queue.end()));
			BRICS_3D::ColoredPointCloud3D *tempPointCloud =  new BRICS_3D::ColoredPointCloud3D();

			for (size_t j = 0; j < seed_queue.size (); ++j) {

				BRICS_3D::ColoredPoint3D *tempPoint =  new BRICS_3D::ColoredPoint3D(
						new BRICS_3D::Point3D(inCloud->getPointCloud()->data()[seed_queue[j]].getX(),
						inCloud->getPointCloud()->data()[seed_queue[j]].getY(),
						inCloud->getPointCloud()->data()[seed_queue[j]].getZ()),
						inCloud->getPointCloud()->data()[seed_queue[j]].red,
						inCloud->getPointCloud()->data()[seed_queue[j]].blue,
						inCloud->getPointCloud()->data()[seed_queue[j]].green);
				tempPointCloud->addPoint(tempPoint);

				delete tempPoint;
			}
			extractedClusters.push_back(tempPointCloud);
			delete tempPointCloud;
		}
		std::cout << "[CHEAT][EuclideanClustering3D] "<< i << " : " << seed_queue.size () << std::endl;
	}

}


int EuclideanClustering3D::segment(){

	assert (this->inputPointCloud!=NULL || this->inputPointCloudColored!=NULL);

	if(isColoredInput){
		extractClusters(this->inputPointCloudColored);
	} else {
		extractClusters(this->inputPointCloud);
	}

	return 1;
}
}
