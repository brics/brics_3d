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

#include "util/PCLTypecaster.h"
#include "core/Logger.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include "EuclideanClusteringPCL.h"

namespace BRICS_3D {

EuclideanClusteringPCL::EuclideanClusteringPCL() {
	this-> minClusterSize =0;
	this->maxClusterSize =0;
	this->clusterTolerance =0;
}

EuclideanClusteringPCL::~EuclideanClusteringPCL() {

}

int EuclideanClusteringPCL::segment(){
	assert (this->inputPointCloud != 0);
	extractClusters(this->inputPointCloud);
	return 1;
}

void EuclideanClusteringPCL::extractClusters(BRICS_3D::PointCloud3D *inCloud){

	BRICS_3D::PCLTypecaster pclTypecaster;
	extractedClusters.clear();
	//converting input cloud into PCL format
	pcl::PointCloud<pcl::PointXYZ>::Ptr inCloudPclPtr(new pcl::PointCloud<pcl::PointXYZ> ());
	pclTypecaster.convertToPCLDataType(inCloudPclPtr, inCloud);

	std::cout << "Ecledian: inCloud size =" << inCloud->getSize() << " inCloudPclPtr size = " << inCloudPclPtr->points.size() << std::endl;

	// Creating the KdTree object for the search method of the extraction
	pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	tree->setInputCloud (inCloudPclPtr);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusterExtractor;
	euclideanClusterExtractor.setClusterTolerance (this->clusterTolerance); // 2cm
	euclideanClusterExtractor.setMinClusterSize (this->minClusterSize);
	euclideanClusterExtractor.setMaxClusterSize (this->maxClusterSize);
	euclideanClusterExtractor.setSearchMethod (tree);
	euclideanClusterExtractor.setInputCloud(inCloudPclPtr);
	euclideanClusterExtractor.extract (cluster_indices);

		printf("Parameters used or extraction:\n \t Min Cluster Size = %d\n"
				"\tMax Cluster Size=%d\n"
			"\tCluster Tolerance=%f\n Number of objects found: %d\n", this->minClusterSize,
				this->maxClusterSize, this->clusterTolerance, cluster_indices.size());
		printf("[EuclideanClusterExtraction.cpp][checkpoint] size of input cloud is %d\n", inCloud->getSize());
	//	//Building up the pointclouds for corresponding clusters
	int index =0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

		extractedClusters.push_back(new BRICS_3D::PointCloud3D());

		//extractedClusters.data()[index]->getPointCloud()->clear();
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			extractedClusters.data()[index]->addPoint(new Point3D(	inCloudPclPtr->points[*pit].x,
					inCloudPclPtr->points[*pit].y,
					inCloudPclPtr->points[*pit].z) );
		}
		printf("[EuclideanClusterExtraction.cpp][checkpoint] size of cluster is %d\n", extractedClusters.data()[index]->getSize());
		index++;
	}
}


}
