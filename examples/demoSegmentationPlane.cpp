/*
 * demoSegmentationPlane.cpp
 *
 *  Created on: Apr 20, 2011
 *      Author: reon
 */

#include "algorithm/segmentation/RegionBasedSACSegmentation.h"
#include "core/PointCloud3D.h"
#include "core/Point3D.h"
#include <iostream>
using namespace std;
int main(){
	//Create a pointcloud object
	BRICS_3D::PointCloud3D cloud;

	//read the points into the pointcloud
	cloud.readFromTxtFile("demoCloud.txt");
	cout<< "INFO: Current PointCloud Size: " <<cloud.getSize()<<endl;

	//Create the vector to hold the model coefficients
	Eigen::VectorXf modelCoefficients;

	//Create the vector to hold the indexes of the model inliers
	std::vector<int> inliers;

	//Create the SACSegmentation Object
	BRICS_3D::RegionBasedSACSegmentation sacSegmenter;
	cout<<"[Checkpoint]: 1 \n";
	//Initialize the segmenter
	sacSegmenter.setDistanceThreshold(0.01);
	sacSegmenter.setInputPointCloud(&cloud);
	sacSegmenter.setMaxIterations(1000);
	sacSegmenter.setMethodType(sacSegmenter.SAC_RANSAC);
	sacSegmenter.setModelType(sacSegmenter.SACMODEL_PLANE);
	sacSegmenter.setOptimizeCoefficients(true);
	sacSegmenter.setProbability(0.99);
	cout<<"[Checkpoint]: 2 \n";
	//Perform the segmentation
	sacSegmenter.segment(inliers,modelCoefficients);
	cout<<"[Checkpoint]: 3 \n";
	if (inliers.size() == 0)
	{
		cout<<"Could not estimate a planar model for the given dataset.";
		return (-1);
	}else {
		cout<<"Found Inliers: " << inliers.size()<<endl;
	}
	cout<<"[Checkpoint]: 4 \n";
	return(1);
}
