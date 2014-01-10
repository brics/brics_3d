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
#include <iostream>
#include "brics_3d/algorithm/featureExtraction/NormalEstimation.h"
#include "brics_3d/algorithm/nearestNeighbor/NearestNeighborANN.h"

int main(){

//This is not a working copy. Will be updated once normal extraction is done.

	//Create a pointcloud object
	brics_3d::PointCloud3D cloud;

	//Create the NormalSet for this cloud
	brics_3d::NormalSet3D normalSet;

	//Create the NormalEstimator object
	brics_3d::NormalEstimation normalEstimator;

	//read the points into the pointcloud
	//Please modify the path if there is a file read error.
	//FIXME do not use absolute path
	cloud.readFromTxtFile("./src/brics_3d/algorithm/segmentation/evaluation/data/demoCloud.txt");
//	cloud.readFromTxtFile("./data/demoCloud.txt");

	if (cloud.getSize()>0){
	cout<< "INFO: Current PointCloud Size: " <<cloud.getSize()<<endl;
	} else {
		cout<< "INFO: Current PointCloud Size: " <<cloud.getSize()<<endl;
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
		cout<<"Could not estimate a model for the given dataset."<<endl;
		return (-1);
	}else {
		cout<<"Found Inliers: " << inliers.size()<<endl;
	}

	cout<<"The model-coefficients are: (" << modelCoefficients[0]<<", " << modelCoefficients[1]<<
			", " << modelCoefficients[2]<<", " << modelCoefficients[3]<<")" <<endl;

	return(1);
}

