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
#include "brics_3d/util/PCLTypecaster.h"
#include "brics_3d/core/ColorSpaceConvertor.h"
#include "brics_3d/core/Logger.h"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "RGBColorBasedEuclideanClustering.h"

namespace brics_3d {

RGBColorBasedEuclideanClustering::RGBColorBasedEuclideanClustering() {
	// TODO Auto-generated constructor stub

}

RGBColorBasedEuclideanClustering::~RGBColorBasedEuclideanClustering() {
	// TODO Auto-generated destructor stub
}


int RGBColorBasedEuclideanClustering::getMaxClusterSize() const
    {
        return maxClusterSize;
    }

    int RGBColorBasedEuclideanClustering::getMinClusterSize() const
    {
        return minClusterSize;
    }

    double RGBColorBasedEuclideanClustering::getToleranceEuclideanDistance() const
    {
        return toleranceEuclideanDistance;
    }

    double RGBColorBasedEuclideanClustering::getToleranceRgbSpace() const
    {
        return toleranceRGBSpace;
    }

    void RGBColorBasedEuclideanClustering::setMaxClusterSize(int maxClusterSize)
    {
        this->maxClusterSize = maxClusterSize;
    }

    void RGBColorBasedEuclideanClustering::setMinClusterSize(int minClusterSize)
    {
        this->minClusterSize = minClusterSize;
    }

    void RGBColorBasedEuclideanClustering::setToleranceEuclideanDistance(double toleranceEuclideanDistance)
    {
        this->toleranceEuclideanDistance = toleranceEuclideanDistance;
    }

    void RGBColorBasedEuclideanClustering::setToleranceRgbSpace(double toleranceRgbSpace)
    {
        toleranceRGBSpace = toleranceRgbSpace;
    }

    int RGBColorBasedEuclideanClustering::segment()
    {
        assert (this->inputPointCloud != 0);
    	if( (inputPointCloud->getSize() > 0)  && ((*inputPointCloud->getPointCloud())[0].asColoredPoint3D() == 0) ) {
    		LOG(ERROR) << "Point clous does not seem to contain color data.";
            return 0;
        }else{
            extractClusters(this->inputPointCloud);
            return 1;
        }
    }

    void RGBColorBasedEuclideanClustering::extractClusters(brics_3d::PointCloud3D *inCloud)
    {
    	this->extractedClusters.clear();
        brics_3d::PCLTypecaster pclTypecaster;
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputXYZ(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>());

        pclTypecaster.convertToPCLDataType(inputXYZ, inCloud);
        pclTypecaster.convertToPCLDataType(inputXYZRGB, inCloud);


        // Creating the KdTree object for the search method of the extraction
        pcl::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        tree->setInputCloud(inputXYZ);
        //============================================================================================
        // Create a bool vector of processed point indices, and initialize it to false
        std::vector<bool> processed(inputXYZ->points.size(), false);
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        // Process all points in the indices vector
        for(size_t i = 0;i < inputXYZ->points.size();++i){
            if(processed[i])
                continue;

            std::vector<int> seed_queue;
            int sq_idx = 0;
            seed_queue.push_back(i);
            processed[i] = true;
            while(sq_idx < (int)(seed_queue.size()))
		{
			// Search for sq_idx
			if (!tree->radiusSearch (seed_queue[sq_idx], this->toleranceEuclideanDistance, nn_indices, nn_distances))
			{
				sq_idx++;
				continue;
			}

			for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
			{
				if (processed[nn_indices[j]]) {
					continue;                            // Has this point been processed before ?
				} else {
					// Perform a simple Euclidean clustering
					float rgbVal24Bit = inputXYZRGB->points[i].rgb;
					uint32_t seed_rgb = *reinterpret_cast<int*> (&rgbVal24Bit);

					rgbVal24Bit = inputXYZRGB->points[nn_indices[j]].rgb;
					uint32_t current_rgb = *reinterpret_cast<int*> (&rgbVal24Bit);

					if(isSimilar(seed_rgb, current_rgb)){	//colors are similar
						//        	      ROS_INFO("Error = %f, RGB_1: [%d, %d, %d], RGB_2: [%d, %d, %d]",
						//     error, rgb_first[0], rgb_first[1], rgb_first[2], rgb_second[0], rgb_second[1], rgb_second[2]);
						seed_queue.push_back (nn_indices[j]);
						processed[nn_indices[j]] = true;
					}
				}
			}

			//        ROS_INFO("Similar color found for : %d neighbors, out of %d popints", count, nn_indices.size());
			sq_idx++;
		}

		// If this queue is satisfactory, add to the clusters
		if (seed_queue.size () >= minClusterSize && seed_queue.size () <= maxClusterSize)
		{
//			std::cout << "[CHEAT][EuclideanClustering3D] found one cluster, size="<< seed_queue.size() << std::endl;

//	        std::cout << "ChekPOINT!! : " << inCloud->getSize() << std::endl;

			seed_queue.erase(std::unique(seed_queue.begin(), seed_queue.end()),seed_queue.end());

//	        std::cout << "ChekPOINT!! : " << inCloud->getSize() << std::endl;

			brics_3d::PointCloud3D *tempPointCloud =  new brics_3d::PointCloud3D();

			for (size_t j = 0; j < seed_queue.size (); ++j) {

				brics_3d::ColoredPoint3D *tempPoint =  new brics_3d::ColoredPoint3D(
						new brics_3d::Point3D(
								(*inputPointCloud->getPointCloud())[seed_queue[j]].getX(),
								(*inputPointCloud->getPointCloud())[seed_queue[j]].getY(),
								(*inputPointCloud->getPointCloud())[seed_queue[j]].getZ()),
								(*inputPointCloud->getPointCloud())[seed_queue[j]].asColoredPoint3D()->getR(),
								(*inputPointCloud->getPointCloud())[seed_queue[j]].asColoredPoint3D()->getG(),
								(*inputPointCloud->getPointCloud())[seed_queue[j]].asColoredPoint3D()->getB());
				tempPointCloud->addPointPtr(tempPoint);

				//				delete tempPoint;
			}
			extractedClusters.push_back(tempPointCloud);
//			delete tempPointCloud; //?
		}

	}

	//================================================================================================

}


} /* namespace brics_3d */
