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

#include "brics_3d/algorithm/segmentation/RegionBasedSACSegmentationUsingNormals.h"
#include "brics_3d/core/PointCloud3D.h"
#include "brics_3d/core/Point3D.h"
#include "brics_3d/core/Logger.h"
#include <iostream>
#include "brics_3d/algorithm/featureExtraction/NormalEstimation.h"
#include "brics_3d/algorithm/nearestNeighbor/NearestNeighborANN.h"

using brics_3d::Logger;

int main(int argc, char **argv) {

//This is not a working copy. Will be updated once normal extraction is done.

	//Configure the logger
	brics_3d::Logger::setMinLoglevel(brics_3d::Logger::INFO);

	//Cofigure input
	string filename;
	if (argc == 2) {
		filename = argv[1];
	} else {
		filename = "../data/segmentation_data/data/demoCloud.txt";
		LOG(INFO) << "No parameter given. Using default data set file: " << filename;
	}

	//Create a pointcloud object
	brics_3d::PointCloud3D cloud;

	//Create the NormalSet for this cloud
	brics_3d::NormalSet3D normalSet;

	//Create the NormalEstimator object
	brics_3d::NormalEstimation normalEstimator;

	//read the points into the pointcloud
	//Please modify the path if there is a file read error.
	cloud.readFromTxtFile(filename);

	if (cloud.getSize()>0){
		LOG(INFO)<< "Current PointCloud Size: " <<cloud.getSize();
	} else {
		LOG(INFO) << "Current PointCloud Size: " <<cloud.getSize();
		return 0;
	}



	//Extract the normals for this pointcloud

	normalEstimator.setInputCloud(&cloud);
	normalEstimator.setSearchMethod(new brics_3d::NearestNeighborANN());
	normalEstimator.setkneighbours(10);
	normalEstimator.computeFeature(&normalSet);

	//Create the vector to hold the model coefficients
	Eigen::VectorXd modelCoefficients;

	//Create the vector to hold the indexes of the model inliers
	std::vector<int> inliers;
	//Create the SACSegmentation Object
	brics_3d::RegionBasedSACSegmentationUsingNormals sacSegmenterUsingNormals;

	//Initialize the segmenter
	sacSegmenterUsingNormals.setPointCloud(&cloud);
	sacSegmenterUsingNormals.setDistanceThreshold(0.01);
	sacSegmenterUsingNormals.setMaxIterations(1000);
	sacSegmenterUsingNormals.setMethodType(sacSegmenterUsingNormals.SAC_RANSAC);
	sacSegmenterUsingNormals.setModelType(sacSegmenterUsingNormals.OBJMODEL_NORMAL_PLANE);
	sacSegmenterUsingNormals.setProbability(0.99);
	sacSegmenterUsingNormals.setInputNormals(&normalSet);


	//Perform the segmentation
	sacSegmenterUsingNormals.segment();
	sacSegmenterUsingNormals.getInliers(inliers);
	sacSegmenterUsingNormals.getModelCoefficients(modelCoefficients);

	if (inliers.size() == 0)
	{
		LOG(INFO) << "Could not estimate a model for the given dataset.";
		return (-1);
	}else {
		cout<<"Found Inliers: " << inliers.size()<<endl;
	}

	LOG(INFO) <<"The model-coefficients are: (" << modelCoefficients[0]<<", " << modelCoefficients[1]<<
			", " << modelCoefficients[2]<<", " << modelCoefficients[3]<<")";

	return(1);
}

