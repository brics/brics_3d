/*
 * demoSegmentationNormalPlane.cpp
 *
 *  Created on: Apr 24, 2011
 *      Author: reon
 */

#include "algorithm/segmentation/RegionBasedSACSegmentationUsingNormals.h"
#include "core/PointCloud3D.h"
#include "core/Point3D.h"
#include <iostream>
#include "algorithm/segmentation/features/NormalEstimation.h"

int main(){

//This is not a working copy. Will be updated once normal extraction is done.

/*	//Create a pointcloud object
	BRICS_3D::PointCloud3D cloud;

	//read the points into the pointcloud
	//Please modify the path if there is a file read error.
	cloud.readFromTxtFile("./trunk/src/algorithm/segmentation/evaluation/data/demoCloud.txt");

	if (cloud.getSize()>0){
	cout<< "INFO: Current PointCloud Size: " <<cloud.getSize()<<endl;
	} else {
		cout<< "INFO: Current PointCloud Size: " <<cloud.getSize()<<endl;
		return 0;
	}


	//Create the vector to hold the model coefficients
	Eigen::VectorXf modelCoefficients;

	//Create the vector to hold the indexes of the model inliers
	std::vector<int> inliers;

	//Create the SACSegmentation Object
	BRICS_3D::RegionBasedSACSegmentation sacSegmenter;

	//Initialize the segmenter
	sacSegmenter.setDistanceThreshold(0.01);
	sacSegmenter.setInputPointCloud(&cloud);
	sacSegmenter.setMaxIterations(1000);
	sacSegmenter.setMethodType(sacSegmenter.SAC_RANSAC);
	sacSegmenter.setModelType(sacSegmenter.OBJMODEL_PLANE);
	sacSegmenter.setOptimizeCoefficients(true);
	sacSegmenter.setProbability(0.99);

	//Perform the segmentation
	sacSegmenter.segment(inliers,modelCoefficients);

	if (inliers.size() == 0)
	{
		cout<<"Could not estimate a planar model for the given dataset."<<endl;
		return (-1);
	}else {
		cout<<"Found Inliers: " << inliers.size()<<endl;
	}

	cout<<"The model-coefficients are: (" << modelCoefficients[0]<<", " << modelCoefficients[1]<<
			", " << modelCoefficients[2]<<", " << modelCoefficients[3]<<")" <<endl;

	return(1);*/
}

